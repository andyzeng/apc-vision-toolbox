#include <random>
#include <algorithm>

float GetRandomFloat(float min, float max) {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(min, max - 0.0001);
    return dist(mt);
}

std::string GetRandomString(size_t str_len) {
    auto rand_char = []() -> char {
        const char char_set[] = "0123456789abcdefghijklmnopqrstuvwxyz";
        return char_set[((int)std::floor(GetRandomFloat(0.0f, (float)sizeof(char_set) - 1)))];
    };
    std::string rand_str(str_len, 0);
    std::generate_n(rand_str.begin(), str_len, rand_char);
    return rand_str;
}


















