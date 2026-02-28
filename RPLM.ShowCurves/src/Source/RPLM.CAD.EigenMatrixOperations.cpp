#include "RPLM.CAD.EigenMatrixOperations.h"

IMatrixOperations::vector2D EigenMatrixOperations::SolveEquation(const vector2D& iCoefficients, const vector2D& iFreeMembers)
{
    // Переводим двумерные векторы в матрицу класса Eigen
    Eigen::MatrixXd coefficientMatrix = ConvertVector2DToEigenMatrix(iCoefficients);
    Eigen::MatrixXd freeTermMatrix = ConvertVector2DToEigenMatrix(iFreeMembers);

    // Решаем СЛАУ
    Eigen::MatrixXd decisionMatrix = Eigen::MatrixXd::Constant(iFreeMembers.size(), iFreeMembers[0].size(), 0);
    decisionMatrix = coefficientMatrix.lu().solve(freeTermMatrix);

    return ConvertEigenMatrixToVector2D(decisionMatrix);
}

RGK::Math::Vector3DArray EigenMatrixOperations::SolveEquationNew(const vector2D& iCoefficients, const RGK::Math::Vector3DArray& iFreeMembers)
{
    // Переводим двумерные векторы в матрицу класса Eigen
    Eigen::MatrixXd coefficientMatrix = ConvertVector2DToEigenMatrix(iCoefficients);
    Eigen::MatrixXd freeTermMatrix = ConvertVector3DArrayToEigenMatrix(iFreeMembers);

    // Решаем СЛАУ
    Eigen::MatrixXd decisionMatrix = coefficientMatrix.lu().solve(freeTermMatrix);

    return ConvertEigenMatrixToVector3DArray(decisionMatrix);
}

int EigenMatrixOperations::GetMatrixRank(const vector2D& iMatrix)
{
    Eigen::MatrixXd matrix = ConvertVector2DToEigenMatrix(iMatrix);
    // Используем LU-разложение
    Eigen::FullPivLU<Eigen::MatrixXd> luDecomp(matrix);
    return static_cast<int>(luDecomp.rank());
}

double EigenMatrixOperations::GetMatrixDet(const vector2D& iMatrix)
{
    Eigen::MatrixXd matrix = ConvertVector2DToEigenMatrix(iMatrix);
    return matrix.determinant();
}

Eigen::MatrixXd EigenMatrixOperations::ConvertVector2DToEigenMatrix(const IMatrixOperations::vector2D& iMatrix)
{
    size_t rows = iMatrix.size();
    size_t cols = iMatrix[0].size();

    Eigen::MatrixXd matrix = Eigen::MatrixXd::Constant(rows, cols, 0);

    for (size_t i = 0; i < rows; ++i)
    {
        for (size_t j = 0; j < cols; ++j)
        {
            matrix(i, j) = iMatrix[i][j];
        }
    }

    return matrix;
}

Eigen::MatrixXd EigenMatrixOperations::ConvertVector3DArrayToEigenMatrix(const RGK::Math::Vector3DArray& iMatrix)
{
    size_t rows = iMatrix.size();
    size_t cols = 3;

    Eigen::MatrixXd matrix = Eigen::MatrixXd::Constant(rows, cols, 0);

    for (size_t i = 0; i < rows; ++i)
    {
        for (size_t j = 0; j < cols; ++j)
        {
            matrix(i, j) = iMatrix[i][j];
        }
    }

    return matrix;
}

RGK::Math::Vector3DArray EigenMatrixOperations::ConvertEigenMatrixToVector3DArray(const Eigen::MatrixXd& iMatrix)
{
    auto rows = iMatrix.rows();
    auto cols = iMatrix.cols();

    RGK::Math::Vector3DArray vec(rows);

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            vec[i][j] = iMatrix(i, j);
        }
    }

    return vec;
}

IMatrixOperations::vector2D EigenMatrixOperations::ConvertEigenMatrixToVector2D(const Eigen::MatrixXd& iMatrix)
{
    auto rows = iMatrix.rows();
    auto cols = iMatrix.cols();

    IMatrixOperations::vector2D vec2D(rows, std::vector<double>(cols));

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            vec2D[i][j] = iMatrix(i, j);
        }
    }

    return vec2D;
}
