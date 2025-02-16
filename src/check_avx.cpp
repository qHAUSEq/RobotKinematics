#include <iostream>
#include <immintrin.h>

int main() { // Простой тест: создаём вектор из четырёх чисел, равных 1.0, // и сохраняем его в массив.
    __m256d a = _mm256_set1_pd(1.0); double result[4]; _mm256_store_pd(result, a);
    std::cout << a << std::endl;
    return 0;
}
