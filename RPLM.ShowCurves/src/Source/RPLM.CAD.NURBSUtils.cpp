#include "RPLM.CAD.NURBSUtils.h"
#include "Geometry/Curves/NURBSCurve.h"

namespace RPLM::CAD
{
    Math::Geometry2D::Geometry::DoubleArray NURBSUtils::FillEvenlyNodalVector(int iDegree, int iNumControlPoints)
    {
        // Кол-во узлов (длина) реальной части узлового вектора
        int _numRealRangeKnots = iNumControlPoints - iDegree + 1;
        // Кол-во узлов в узл. векторе
        int numKnots = iNumControlPoints + iDegree + 1;

        // Начало/конец реального диапазона узл. вектора
        int realRangeStart = iDegree;
        int realRangeEnd = numKnots - iDegree - 1;

        RPLM::Math::Geometry2D::Geometry::DoubleArray knots(numKnots);

        // Шаг в реальном диапазоне
        double step = 1 / static_cast<double>(_numRealRangeKnots - 1);

        // Заполняем реальный диапазон
        for (int i = realRangeStart + 1; i < realRangeEnd; ++i)
            knots[i] = knots[i - 1] + step;

        // Заполняем последние параметры единицами
        for (size_t i = realRangeEnd; i < knots.size(); ++i)
            knots[i] = 1;

        return knots;
    }

    Math::Geometry2D::Geometry::DoubleArray NURBSUtils::FillDefaultNodalVector(int iDegree, int iNumControlPoints)
    {
        // Кол-во узлов (длина) реальной части узлового вектора
        int _numRealRangeKnots = iNumControlPoints - iDegree + 1;
        // Кол-во узлов в узл. векторе
        int numKnots = iNumControlPoints + iDegree + 1;

        // Начало/конец реального диапазона узл. вектора
        int realRangeStart = iDegree;
        int realRangeEnd = numKnots - iDegree - 1;

        RPLM::Math::Geometry2D::Geometry::DoubleArray knots(numKnots);

        // Заполняем реальный диапазон
        for (int i = realRangeStart + 1; i < realRangeEnd; ++i)
            knots[i] = 0.5;

        // Заполняем последние параметры единицами
        for (size_t i = realRangeEnd; i < knots.size(); ++i)
            knots[i] = 1;

        return knots;
    }

    int NURBSUtils::FindSpanForParameter(const std::vector<double>& iNodalVector, int iDegree, double iCurveParameter)
    {
        size_t numKnots = iNodalVector.size();

        if (iCurveParameter < iNodalVector[iDegree] || iCurveParameter > iNodalVector[numKnots - iDegree - 1])
        {
            throw _STR("Ошибка в функции findSpanForParameter: параметр вышел за реальный диапазон!");
        }

        if (iCurveParameter == iNodalVector[numKnots - iDegree - 1])
        {
            return static_cast<int>(numKnots) - iDegree - 2;
        }

        int low = iDegree;
        int high = static_cast<int>(numKnots) - iDegree - 1;
        int middle = (low + high) / 2;

        while ((iCurveParameter < iNodalVector[middle]) || (iCurveParameter >= iNodalVector[middle + 1]))
        {
            if (iCurveParameter < iNodalVector[middle])
            {
                high = middle;
            }
            else
            {
                low = middle;
            }

            middle = (low + high) / 2;
        }

        return middle;
    }

    void NURBSUtils::CalculateZeroBasisFuncs(RGK::Vector<RGK::Vector<double>>& iBasisFuncs, RGK::Vector<RGK::Vector<double>>& iTempStorage, RPLM::Math::Geometry2D::Geometry::DoubleArray& iNodalVector, int iDegree, double iCurveParameter)
    {
        int span = FindSpanForParameter(iNodalVector, iDegree, iCurveParameter);

        RGK::Vector<double> left(iDegree + 1), right(iDegree + 1);
        iTempStorage[0][0] = 1;

        for (int i = 1; i < iDegree + 1; ++i)
        {
            left[i] = iCurveParameter - iNodalVector[span + 1 - i];
            right[i] = iNodalVector[span + i] - iCurveParameter;
            double saved = 0;

            for (int j = 0; j < i; ++j)
            {
                // Нижний треугольник
                iTempStorage[i][j] = right[j + 1] + left[i - j];
                double temp = iTempStorage[j][i - 1] / iTempStorage[i][j];
                // Верхний треугольник
                iTempStorage[j][i] = saved + right[j + 1] * temp;
                saved = left[i - j] * temp;
            }

            iTempStorage[i][i] = saved;
        }

        for (int i = 0; i <= iDegree; ++i)
        {
            iBasisFuncs[0][i] = iTempStorage[i][iDegree];
        }
    }

    void NURBSUtils::CalculateDerivsBasisFuncs(RGK::Vector<RGK::Vector<double>>& iBasisFuncs, RGK::Vector<RGK::Vector<double>>& iTempStorage, int iDegree)
    {
        // Хранит два вычисленных ряда
        RGK::Vector<RGK::Vector<double>> rows(2, RGK::Vector<double>(iDegree + 1));

        for (int i = 0; i < iDegree + 1; ++i)
        {
            int s1 = 0, s2 = 1;
            rows[0][0] = 1;

            for (int j = 1; j <= iDegree; ++j)
            {
                double d = 0;
                int rk = i - j;
                int pk = iDegree - j;

                if (i >= j)
                {
                    rows[s2][0] = rows[s1][0] / iTempStorage[pk + 1][rk];
                    d = rows[s2][0] * iTempStorage[rk][pk];
                }

                int j1 = 0, j2 = 0;

                if (rk >= -1)
                {
                    j1 = 1;
                }
                else
                {
                    j1 = -rk;
                }

                if (i - 1 <= pk)
                {
                    j2 = j - 1;
                }
                else
                {
                    j2 = iDegree - i;
                }

                for (int j = j1; j <= j2; ++j)
                {
                    rows[s2][j] = (rows[s1][j] - rows[s1][j - 1]) / iTempStorage[pk + 1][rk + j];
                    d += rows[s2][j] * iTempStorage[rk + j][pk];
                }

                if (i <= pk)
                {
                    rows[s2][j] = -rows[s1][j - 1] / iTempStorage[pk + 1][i];
                    d += rows[s2][j] * iTempStorage[i][pk];
                }

                iBasisFuncs[j][i] = d;
                int temp = s1;
                s1 = s2;
                s2 = temp;
            }
        }

        double k = iDegree;

        // Умножаем на коэффициенты
        for (int i = 1; i <= iDegree; ++i)
        {
            for (int j = 0; j < iDegree + 1; ++j)
            {
                iBasisFuncs[i][j] *= k;
            }

            k *= iDegree - i;
        }
    }

    RGK::Vector<RGK::Vector<double>> NURBSUtils::CalculateBasisFuncs(const RGK::NURBSCurve& iCurve, double iCurveParameter)
    {
        int degree = iCurve.GetDegree();

        // Для хранения базисных функций и их производных (нулевая строка - нулевые производные, первая строка - первые произв. и т.д.)
        RGK::Vector<RGK::Vector<double>> basisFuncs(degree + 1, RGK::Vector<double>(degree + 1));
        // Для хранения базисных функций и узлов различия
        RGK::Vector<RGK::Vector<double>> tempStorage(degree + 1, RGK::Vector<double>(degree + 1));

        // Вычисляем нулевые базисные функции
        CalculateZeroBasisFuncs(basisFuncs, tempStorage, iCurve.GetKnots(), degree, iCurveParameter);
        // Вычисляем все производные базисных функций
        CalculateDerivsBasisFuncs(basisFuncs, tempStorage, degree);

        double sum = 0;

        // Для контроля суммируем значения базисных функций в точке
        for (int i = 0; i < degree + 1; ++i)
        {
            sum += basisFuncs[0][i];
        }

        // Сумма базисных функций должна = 1
        if (sum < (1 - 1e-10) || (sum > 1 + 1e-10))
        {
            throw _STR("Ошибка в функции calculateBasisFuncs: сумма базисных функций не равна 1!");
        }

        return basisFuncs;
    }
}
