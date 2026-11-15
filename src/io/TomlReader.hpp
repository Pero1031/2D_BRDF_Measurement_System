/*
* @file src/io/TomlReader.hpp
* @brief Small helper for reading TOML tables with clear, key-named error messages. (internal)
*/

#pragma once

#include <array>
#include <cmath>
#include <format>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include <toml++/toml.hpp>

#include "brdf/core/Algebra.hpp"
#include "brdf/core/Error.hpp"

namespace brdf::io::detail {

#define BRDF_TRY(expr)                                   \
    do {                                                 \
        if (auto brdf_try_status_ = (expr); !brdf_try_status_) \
            return tl::unexpected(brdf_try_status_.error()); \
    } while (0)

    /// TOML のテーブル 1 つを読むための道具。
    /// - キーが無ければ既定値のまま(何もしない)。型が違えば「どのファイルのどのセクションのどのキーか」を示すエラー。
    /// - finish() で、読まれなかったキー(= 綴り間違いの可能性)をエラーにする。
    class Section {
    public:
        Section(const toml::table* table, std::string origin, std::string path)
            : table_(table), origin_(std::move(origin)), path_(std::move(path)) {}

        bool present() const { return table_ != nullptr; }
        bool has(std::string_view key) const { return table_ && table_->contains(key); }
        const std::string& origin() const { return origin_; }
        const std::string& path() const { return path_; }

        Status number(std::string_view key, Real& out) {
            const toml::node* n = find(key);
            if (!n) return {};
            auto v = n->value<double>();
            if (!v || !std::isfinite(*v)) return bad(key, "must be a number");
            out = *v;
            return {};
        }

        Status integer(std::string_view key, int& out) {
            const toml::node* n = find(key);
            if (!n) return {};
            auto v = n->value<int64_t>();
            if (!v) return bad(key, "must be an integer");
            out = static_cast<int>(*v);
            return {};
        }

        Status boolean(std::string_view key, bool& out) {
            const toml::node* n = find(key);
            if (!n) return {};
            auto v = n->value<bool>();
            if (!v) return bad(key, "must be true or false");
            out = *v;
            return {};
        }

        Status string(std::string_view key, std::string& out) {
            const toml::node* n = find(key);
            if (!n) return {};
            auto v = n->value<std::string>();
            if (!v) return bad(key, "must be a string");
            out = *v;
            return {};
        }

        /// 数値が N 個の配列。
        template <std::size_t N>
        Status numbers(std::string_view key, std::array<Real, N>& out) {
            const toml::node* n = find(key);
            if (!n) return {};
            const toml::array* a = n->as_array();
            if (!a || a->size() != N) {
                return bad(key, "must be an array of " + std::to_string(N) + " numbers");
            }
            std::array<Real, N> tmp{};
            for (std::size_t i = 0; i < N; ++i) {
                auto v = a->get(i)->value<double>();
                if (!v || !std::isfinite(*v)) return bad(key, "must be an array of " + std::to_string(N) + " numbers");
                tmp[i] = *v;
            }
            out = tmp;
            return {};
        }

        Status vec3(std::string_view key, Vec3& out) {
            std::array<Real, 3> a{out.x(), out.y(), out.z()};
            BRDF_TRY(numbers(key, a));
            out = Vec3(a[0], a[1], a[2]);
            return {};
        }

        Status vec2(std::string_view key, Vec2& out) {
            std::array<Real, 2> a{out.x(), out.y()};
            BRDF_TRY(numbers(key, a));
            out = Vec2(a[0], a[1]);
            return {};
        }

        Status stringArray(std::string_view key, std::vector<std::string>& out) {
            const toml::node* n = find(key);
            if (!n) return {};
            const toml::array* a = n->as_array();
            if (!a) return bad(key, "must be an array of strings");
            std::vector<std::string> tmp;
            for (const toml::node& e : *a) {
                auto v = e.value<std::string>();
                if (!v) return bad(key, "must be an array of strings");
                tmp.push_back(*v);
            }
            out = std::move(tmp);
            return {};
        }

        /// 子テーブル。無ければ「存在しない Section」を返す(その中のキーはすべて既定値になる)。
        Result<Section> child(std::string_view key) {
            const toml::node* n = find(key);
            if (!n) return Section(nullptr, origin_, join(key));
            const toml::table* t = n->as_table();
            if (!t) return fail(ErrorCode::InvalidArgument, origin_ + ": [" + join(key) + "] must be a table");
            return Section(t, origin_, join(key));
        }

        /// テーブルの配列 ( [[name]] )。無ければ空。
        Result<std::vector<Section>> tableArray(std::string_view key) {
            std::vector<Section> out;
            const toml::node* n = find(key);
            if (!n) return out;
            const toml::array* a = n->as_array();
            if (!a) return fail(ErrorCode::InvalidArgument, origin_ + ": " + join(key) + " must be an array of tables ([[" + std::string(key) + "]])");
            int i = 0;
            for (const toml::node& e : *a) {
                const toml::table* t = e.as_table();
                if (!t) return fail(ErrorCode::InvalidArgument, origin_ + ": " + join(key) + " must be an array of tables");
                out.emplace_back(t, origin_, join(key) + "[" + std::to_string(++i) + "]");
            }
            return out;
        }

        /// 読まれなかったキーがあればエラー(綴り間違いの検出)。
        Status finish() {
            if (!table_) return {};
            for (auto&& [k, v] : *table_) {
                (void)v;
                if (!used_.contains(std::string(k.str()))) {
                    return fail(ErrorCode::InvalidArgument,
                                origin_ + ": unknown key '" + std::string(k.str()) + "' in " + where());
                }
            }
            return {};
        }

        tl::unexpected<Error> bad(std::string_view key, const std::string& what) const {
            return fail(ErrorCode::InvalidArgument, origin_ + ": " + where() + " " + std::string(key) + " " + what);
        }

    private:
        const toml::node* find(std::string_view key) {
            used_.insert(std::string(key));
            return table_ ? table_->get(key) : nullptr;
        }
        std::string join(std::string_view key) const {
            return path_.empty() ? std::string(key) : path_ + "." + std::string(key);
        }
        std::string where() const { return path_.empty() ? "(top level)" : "[" + path_ + "]"; }

        const toml::table* table_;
        std::string origin_;
        std::string path_;
        std::set<std::string> used_;
    };

    /// 文字列を TOML として解析する。失敗したら「ファイル名:行:列」つきのエラー。
    inline Result<toml::table> parseToml(std::string_view text, std::string_view origin) {
#if TOML_EXCEPTIONS
        try {
            return toml::parse(text, origin);
        } catch (const toml::parse_error& e) {
            return fail(ErrorCode::InvalidArgument,
                        std::string(origin) + ":" + std::to_string(e.source().begin.line) + ":"
                        + std::to_string(e.source().begin.column) + ": " + std::string(e.description()));
        }
#else
        toml::parse_result r = toml::parse(text, origin);
        if (!r) {
            return fail(ErrorCode::InvalidArgument,
                        std::string(origin) + ":" + std::to_string(r.error().source().begin.line) + ":"
                        + std::to_string(r.error().source().begin.column) + ": " + std::string(r.error().description()));
        }
        return std::move(r).table();
#endif
    }

    // --- 書き出し用の整形 ---

    /// 数値を TOML の浮動小数として書く(整数に見えないよう ".0" を付ける)。
    inline std::string num(double v) {
        std::string s = std::format("{:.12g}", v);
        if (s.find_first_of(".eEn") == std::string::npos) s += ".0";
        return s;
    }

    inline std::string arr3(const Vec3& v) {
        return "[" + num(v.x()) + ", " + num(v.y()) + ", " + num(v.z()) + "]";
    }

    inline std::string quote(const std::string& s) {
        std::string out = "\"";
        for (char c : s) {
            if (c == '"' || c == '\\') out += '\\';
            if (c == '\n') { out += "\\n"; continue; }
            out += c;
        }
        return out + "\"";
    }

} // namespace brdf::io::detail
