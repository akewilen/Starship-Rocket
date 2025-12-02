#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Matrix2d A;
    A << 1, 2,
         3, 4;

    std::cout << "A =\n" << A << std::endl;
}
