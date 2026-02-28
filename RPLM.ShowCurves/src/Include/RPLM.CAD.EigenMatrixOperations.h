#pragma once

#include "RPLM.CAD.IMatrixOperations.h"
// Использую Eigen, подключенный к данному проекту
#include "Eigen\Dense"


#include <Geometry/Curves/NURBSCurve.h>

class EigenMatrixOperations final : public IMatrixOperations
{
public:
    /// <summary>Решает СЛАУ</summary>
    /// <param name="iCoefficients">Матрица коэффициентов</param>
    /// <param name="iFreeMembers">Свободные члены</param>
    /// <returns>Решение СЛАУ</returns>
    vector2D SolveEquation(const vector2D& iCoefficients, const vector2D& iFreeMembers) override;

    RGK::Math::Vector3DArray SolveEquationNew(const vector2D& iCoefficients, const RGK::Math::Vector3DArray& iFreeMembers) override;

    /// <summary>Возвращает ранг матрицы</summary>
    /// <param name="iMatrix">Матрица</param>
    /// <returns>Ранг матрицы</returns>
    int GetMatrixRank(const vector2D& iMatrix) override;

    /// <summary>Возвращает определитель матрицы</summary>
    /// <param name="iMatrix">Матрица</param>
    /// <returns>Определитель матрицы</returns>
    double GetMatrixDet(const vector2D& iMatrix) override;

private:
    /// <summary>Конвертирует двумерный вектор в матрицу класса Eigen</summary>
    /// <param name="iMatrix">Двумерный вектор</param>
    /// <returns>Матрица класса Eigen</returns>
    Eigen::MatrixXd ConvertVector2DToEigenMatrix(const IMatrixOperations::vector2D& iMatrix);

    Eigen::MatrixXd ConvertVector3DArrayToEigenMatrix(const RGK::Math::Vector3DArray& iMatrix);

    RGK::Math::Vector3DArray ConvertEigenMatrixToVector3DArray(const Eigen::MatrixXd& iMatrix);

    /// <summary>Конвертирует матрицу класса Eigen в двумерный вектор</summary>
    /// <param name="iMatrix">Матрица класса Eigen</param>
    /// <returns>Двумерный вектор</returns>
    IMatrixOperations::vector2D ConvertEigenMatrixToVector2D(const Eigen::MatrixXd& iMatrix);
};
