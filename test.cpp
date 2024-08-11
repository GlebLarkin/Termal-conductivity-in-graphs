#include <Eigen/Dense>
#include <iostream>

int main() {
    Eigen::MatrixXd A(2, 2);
    A(0, 0) = 1;
    A(1, 0) = 2;
    A(0, 1) = 3;
    A(1, 1) = 4;

    std::cout << "Here is the matrix A:\n" << A << std::endl;

    Eigen::EigenSolver<Eigen::MatrixXd> solver(A);
    std::cout << "Eigenvalues:\n" << solver.eigenvalues() << std::endl;
    std::cout << "Eigenvectors:\n" << solver.eigenvectors() << std::endl;

    return 0;
}
