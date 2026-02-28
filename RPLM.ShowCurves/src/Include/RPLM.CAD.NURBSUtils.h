#pragma once

#include "RPLM.GeomCore/RPLM.Math.Geometry2D/Geometry/RGPGeometryForward.h"

namespace RPLM::CAD
{
    // Класс с методами для взаимодействия с NURBS
    class NURBSUtils
    {
    public:
        /// <summary>Равномерно заполняет узловой вектор кривой</summary>
        /// <param name="iDegree">Степень кривой</param>
        /// <param name="iNumControlPoints">Число контрольных точек кривой</param>
        /// <returns>Равномерно заполненный узловой вектор</returns>
        static Math::Geometry2D::Geometry::DoubleArray FillEvenlyNodalVector(int iDegree, int iNumControlPoints);

        /// <summary>Заполняет узловой вектор кривой по умолчанию (0, ..., 0, 0.5, ..., 0.5, 1, ..., 1)</summary>
        /// <param name="iDegree">Степень кривой</param>
        /// <param name="iNumControlPoints">Число контрольных точек кривой</param>
        /// <returns>Равномерно заполненный узловой вектор</returns>
        static Math::Geometry2D::Geometry::DoubleArray FillDefaultNodalVector(int iDegree, int iNumControlPoints);

        /// <summary>Находит номер интервала (спан) для заданного параметра кривой</summary>
        /// <param name="iNodalVector">Узловой вектор</param>
        /// <param name="iDegree">Степень кривой</param>
        /// <param name="iCurveParameter">Параметр кривой</param>
        /// <returns>Номер интервала (спан)</returns>
        static int FindSpanForParameter(const std::vector<double>& iNodalVector, int iDegree, double iCurveParameter);

        /// <summary>Вычисляет нулевые базисные функции</summary>
        /// <param name="iBasisFuncs">Вектор базисных функций</param>
        /// <param name="iTempStorage">Временный вектор для сбора значений базисных функций</param>
        /// <param name="iNodalVector">Узловой вектор</param>
        /// <param name="iDegree">Степень кривой</param>
        /// <param name="iCurveParameter">Параметр кривой</param>
        static void CalculateZeroBasisFuncs(RGK::Vector<RGK::Vector<double>>& iBasisFuncs, RGK::Vector<RGK::Vector<double>>& iTempStorage, RPLM::Math::Geometry2D::Geometry::DoubleArray& iNodalVector, int iDegree, double iCurveParameter);

        /// <summary>Вычисляет производные базисных функций</summary>
        /// <param name="iBasisFuncs">Базисные функции</param>
        /// <param name="iTempStorage">Временный вектор для сбора значений базисных функций</param>
        /// <param name="iDegree">Степень</param>
        static void CalculateDerivsBasisFuncs(RGK::Vector<RGK::Vector<double>>& iBasisFuncs, RGK::Vector<RGK::Vector<double>>& iTempStorage, int iDegree);

        /// <summary>Вычисляет базисные функции и их производные кривой</summary>
        /// <param name="iCurve">Кривая</param>
        /// <param name="iCurveParameter">Параметр кривой, в котором будут рассчитываться базисные функции</param>
        /// <returns>Вектор базисных функций, где нулевая строка - нулевые производные,
        /// первая строка - первые производные и т.д.</returns>
        static RGK::Vector<RGK::Vector<double>> CalculateBasisFuncs(const RGK::NURBSCurve& iCurve, double iCurveParameter);
    };
}
