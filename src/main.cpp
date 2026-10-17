#include <iostream>
#include <string_view>

int main(int argc, char* argv[]) {
    if (argc == 2 && std::string_view(argv[1]) == "--version") {
        std::cout << "vehiclesim " << VEHICLESIM_VERSION << '\n';
        return 0;
    }
    std::cerr << "usage: vehiclesim --version\n";
    return 2;
}
