#pragma once

#include <vector>
#include <memory>
#include <Common/Math/Vector3D.h>

class IMatrixOperations;
using IMatrixOperationsPtr = std::shared_ptr<IMatrixOperations>;

// Содержит названия библиотек для работы с матрицами
enum OperationClass
{
    eigen
};

class IMatrixOperations
{
public:
    using vector2D = std::vector<std::vector<double>>;

    /// <summary>Решает СЛАУ</summary>
    /// <param name="iCoefficients">Матрица коэффициентов</param>
    /// <param name="iFreeMembers">Свободные члены</param>
    /// <returns>Решение СЛАУ</returns>
    virtual vector2D SolveEquation(const vector2D& iCoefficients, const vector2D& iFreeMembers) = 0;

    virtual RGK::Math::Vector3DArray SolveEquationNew(const vector2D& iCoefficients, const RGK::Math::Vector3DArray& iFreeMembers) = 0;

    /// <summary>Возвращает ранг матрицы</summary>
    /// <param name="iMatrix">Матрица</param>
    /// <returns>Ранг матрицы</returns>
    virtual int GetMatrixRank(const vector2D& iMatrix) = 0;

    /// <summary>Возвращает определитель матрицы</summary>
    /// <param name="iMatrix">Матрица</param>
    /// <returns>Определитель матрицы</returns>
    virtual double GetMatrixDet(const vector2D& iMatrix) = 0;

    /// <summary>Возвращает текущую выбранную библиотеку для решения СЛАУ</summary>
    /// <param name="iClassName">Название библиотеки</param>
    /// <returns>Указатель на библиотеку</returns>
    static IMatrixOperationsPtr GetMatrixOperationsClass(OperationClass iClassName);
};
