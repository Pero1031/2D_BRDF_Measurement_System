#include <toml++/toml.h>
#include <iostream>

int main() {
    toml::table tbl = toml::parse(R"(
        title = "BRDF Recipe"
        [sensor]
        integration_ms = 50
        average = 8
    )");

    std::cout << tbl["title"].value_or("none") << "\n";
    std::cout << tbl["sensor"]["integration_ms"].value_or(0) << "\n";
}
