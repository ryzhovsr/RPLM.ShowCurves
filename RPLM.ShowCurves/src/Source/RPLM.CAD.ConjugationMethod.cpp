#include "RPLM.CAD.ConjugationMethod.h"
#include "RPLM.CAD.IMatrixOperations.h"
#include "RPLM.CAD.NURBSUtils.h"
#include "RGPSession.h"

namespace RPLM::CAD::ConjugationCurves
{
    RGK::NURBSCurve ConjugationMethod::ConjugateCurve(const RGK::NURBSCurve& iCurve, bool iFixBeginningCurve = false, bool iFixEndCurve = false)
    {
        // Разбиваем NURBS кривую на кривые Безье
        RGK::Vector<RGK::NURBSCurve> bezierCurves = DivideCurveIntoBezierCurves(iCurve);

        // Вычисляем базисные функции и их производные оригинальной кривой в параметре 1
        double curveParameter = 1;
        RGK::Vector<RGK::Vector<double>> basisFuncs = NURBSUtils::CalculateBasisFuncs(bezierCurves[0], curveParameter);

        const size_t NUMBER_BASIS_FUNCS = static_cast<size_t>(iCurve.GetDegree()) + 1;                      // Количество базисных функций
        const size_t NUMBER_BEZIER_CURVES = bezierCurves.size();                                            // Количество кривых Безье
        const size_t NUMBER_BREAK_POINTS = NUMBER_BEZIER_CURVES - 1;                                        // Количество потенциальных точек разрыва между кривыми
        const size_t NUMBER_EPSILONS = bezierCurves.size() * bezierCurves[0].GetControlPoints().size();     // Количество эпсилон, которые будут регулировать контрольные точки
        const size_t MATRIX_SIZE = NUMBER_BASIS_FUNCS * (2 * NUMBER_BEZIER_CURVES - 1);                     // Размер матрицы коэффициентов

        // Матрица коэффициентов
        RGK::Vector<RGK::Vector<double>> coefficientMatrix(MATRIX_SIZE, RGK::Vector<double>(MATRIX_SIZE));

        // Заполняем матрицу коэффициентов
        FillCoefficientsMatrix(coefficientMatrix, basisFuncs, NUMBER_EPSILONS, NUMBER_BREAK_POINTS);

        // Фиксируем начало и конец кривой
        FixateCurve(coefficientMatrix, NUMBER_EPSILONS, NUMBER_BASIS_FUNCS, iFixBeginningCurve, iFixEndCurve);

        // Контрольные точки кривых Безье
        RGK::Vector<RGK::Math::Vector3DArray> controlPointsBezierCurves(NUMBER_BEZIER_CURVES);

        for (size_t i = 0; i != NUMBER_BEZIER_CURVES; ++i)
        {
            controlPointsBezierCurves[i] = bezierCurves[i].GetControlPoints();
        }

        // Матрица свободных членов. RGK::Vector<double>(3) - потому что 3 координаты x, y, z. Можно исправить в дальнейшем, если узнать тип данных для точки
        RGK::Vector<RGK::Vector<double>> freeMembersMatrix(MATRIX_SIZE, RGK::Vector<double>(3));

        // Вычисляем реверсивные базисные функции и их производные в параметре 0
        curveParameter = 0;
        RGK::Vector<RGK::Vector<double>> reverseBasisFuncs = NURBSUtils::CalculateBasisFuncs(bezierCurves[0], curveParameter);

        // Заполняем матрицу свободных членов
        FillFreeMemberMatrix(freeMembersMatrix, controlPointsBezierCurves, basisFuncs, reverseBasisFuncs, NUMBER_EPSILONS);

        // Вычисляем точки смещения для новых контрольных точек
        RGK::Vector<RGK::Vector<double>> shiftPoints = CalculateShiftPoints(coefficientMatrix, freeMembersMatrix);

        // Делаем сдвиг исходных контрольных точек для сопряжения
        AdjustControlPoints(controlPointsBezierCurves, shiftPoints, NUMBER_BEZIER_CURVES);

        // Вычисляем новые кривые Безье, которые будут непрерывны
        RGK::Vector<RGK::NURBSCurve> newBezierCurves = CreateBezierCurves(controlPointsBezierCurves, NUMBER_BEZIER_CURVES, bezierCurves[0].GetDegree());

        // Представляем вектор кривых Безье как одну кривую NURBS
        RGK::NURBSCurve merdgedCurve = BezierCurvesToNURBS(newBezierCurves, bezierCurves[0].GetDegree());

        return merdgedCurve;
    }

    RGK::NURBSCurve ConjugationMethod::ConjugateCurves(const RGK::NURBSCurve& iCurve1, const RGK::NURBSCurve& iCurve2, bool iFixBeginningCurve, bool iFixEndCurve)
    {
        if (!iCurve1 || !iCurve2)
            return nullptr;

        RGK::Math::Vector3DArray controlPoints1 = iCurve1.GetControlPoints();
        RGK::Math::Vector3DArray controlPoints2 = iCurve2.GetControlPoints();
        // Кол-во контрольных точек
        int numControlPoints = 0;

        if (controlPoints1.size() != controlPoints2.size())
            return nullptr;
        else
            numControlPoints = controlPoints1.size();

        // Производные первой кривой
        RGK::Math::Vector3DArray derivs1(numControlPoints);
        // Отрицательные дельты контрольных точек первой кривой
        RGK::Math::Vector3DArray negDerivs1(numControlPoints);
        // Стартовый индекс начинается с конца
        int startIndex1 = numControlPoints - 1;

        // Находим дельты для первой кривой
        for (int i = numControlPoints - 1; i >= 0; --i)
        {
            derivs1[numControlPoints - i - 1] = calcDerivLeftBezierCurveForMerger(controlPoints1, i, startIndex1);
            negDerivs1[numControlPoints - i - 1] = calcNegativeDerivLeftBezierCurveForMerger(controlPoints1, i, startIndex1);
        }

        // Производные второй кривой
        RGK::Math::Vector3DArray derivs2(numControlPoints);
        // Стартовый индекс начинается с начала
        int startIndex2 = 0;

        // Находим дельты для второй кривой
        for (int i = 0; i < numControlPoints; ++i)
            derivs2[i] = calcDerivRightBezierCurveForMerger(controlPoints2, i, startIndex2);

        // Матрица коэффициентов
        std::vector<std::vector<double>> coefficients(numControlPoints * 3, std::vector<double>(numControlPoints * 3));

        // Заполняем матрицу коэффициентов
        for (int i = 0; i < numControlPoints * 2; ++i)
            coefficients[i][i] = 2;

        for (int i = 0; i < numControlPoints; ++i)
        {
            int resNum = 0;
            int counter = 0;

            for (int r = 0; r <= i; ++r)
            {
                resNum = pow(-1, r - i) * CalcCombWithoutRepetition(i, r);
                coefficients[numControlPoints * 2 + i][startIndex1 - i + counter] = resNum;
                coefficients[numControlPoints * 2 + i][startIndex1 + i - counter + 1] = pow(-1, i % 2 ? 0 : 1) * resNum;

                coefficients[startIndex1 - i + counter][numControlPoints * 2 + i] = resNum;
                coefficients[startIndex1 + i - counter + 1][numControlPoints * 2 + i] = pow(-1, i % 2 ? 0 : 1) * resNum;
                ++counter;
            }
        }

        if (iFixBeginningCurve)
        {
            coefficients[0][0] = 1;
            coefficients[numControlPoints * 2 - 1][numControlPoints * 2 - 1] = 1;
        }

        if (iFixEndCurve)
        {
            coefficients[0][coefficients[0].size() - 1] = 0;
            coefficients[numControlPoints * 2 - 1][coefficients[0].size() - 1] = 0;
        }

        // Матрица свободных членов
        RGK::Math::Vector3DArray freeMembers(numControlPoints * 3);
        int counter = 0;

        // Заполняем матрицу свободных членов
        for (int i = numControlPoints * 2; i < numControlPoints * 3; ++i)
        {
            freeMembers[i] = negDerivs1[counter] + derivs2[counter];
            ++counter;
        }

        auto operation = IMatrixOperations::GetMatrixOperationsClass(OperationClass::eigen);

        if (operation == nullptr)
            throw "Error! _calculateShiftPoints: operation = nullptr";

        RGK::Math::Vector3DArray solution = operation->SolveEquationNew(coefficients, freeMembers);

        // Временные точки для расчёта новых контрольных точек
        RGK::Math::Vector3DArray tempPoints(numControlPoints);
        RGK::Math::Vector3DArray controlPointsNewCurve(numControlPoints);

        for (int i = 0; i < numControlPoints; ++i)
        {
            tempPoints[i] = (controlPoints1[i] + solution[i]);
        }

        controlPointsNewCurve[0] = tempPoints[0];
        counter = 1;

        while (counter != numControlPoints)
        {
            for (int i = 0; i < numControlPoints - counter; ++i)
                tempPoints[i] = -1 * tempPoints[i] + 2 * tempPoints[i + 1];

            controlPointsNewCurve[counter] = tempPoints[0];
            ++counter;
        }

        // Найдём новые точки двух кривых по отдельности
        RGK::Math::Vector3DArray newControlPoints1(numControlPoints);
        RGK::Math::Vector3DArray newControlPoints2(numControlPoints);

        for (int i = 0; i < numControlPoints; ++i)
        {
            newControlPoints1[i] = (controlPoints1[i] + solution[i]);
            newControlPoints2[i] = (controlPoints2[i] + solution[numControlPoints + i]);
        }

        // Для сплайна из новых точек - было 11 и станет 11 контрольных точек
        RGK::Math::Vector3DArray answer(newControlPoints1.size() * 2 - 1);
        std::vector<double> weights(answer.size(), 1);

        for (int i = 0; i < newControlPoints1.size(); ++i)
        {
            answer[i] = newControlPoints1[i];
            answer[newControlPoints1.size() - 1 + i] = newControlPoints2[i];
        }

        // Находим новые производные для первой кривой
        for (int i = numControlPoints - 1; i >= 0; --i)
        {
            derivs1[numControlPoints - i - 1] = calcDerivLeftBezierCurveForMerger(newControlPoints1, i, startIndex1);
            negDerivs1[numControlPoints - i - 1] = calcNegativeDerivLeftBezierCurveForMerger(newControlPoints1, i, startIndex1);
        }

        for (int i = 0; i < numControlPoints; ++i)   // Находим новые производные для второй кривой
            derivs2[i] = calcDerivRightBezierCurveForMerger(newControlPoints2, i, startIndex2);

        for (int i = 0; i < numControlPoints; ++i)
        {
            if (abs(derivs1[i].GetX() - derivs2[i].GetX()) > 0.001 || abs(derivs1[i].GetY() - derivs2[i].GetY()) > 0.001 || abs(derivs1[i].GetZ() - derivs2[i].GetZ()) > 0.001)
            {
                throw "Error! attachCurvesUsualMethod: derivsFirstCurve[i] != derivsSecondCurve[i]";
            }
        }

        RGK::Context rgkContext;
        RPLM::EP::Model::Session::GetSession()->GetRGKSession().CreateMainContext(rgkContext);

        Math::Geometry2D::Geometry::DoubleArray knots = NURBSUtils::FillDefaultNodalVector(iCurve1.GetDegree(), static_cast<int>(answer.size()));

        RGK::NURBSCurve newCurve;
        RGK::NURBSCurve::Create(rgkContext, answer, iCurve1.GetDegree(), knots, false, newCurve);

        return newCurve;
    }

    RGK::Vector<RGK::NURBSCurve> ConjugationMethod::DivideCurveIntoBezierCurves(const RGK::NURBSCurve& iCurve)
    {
        // Контрольные точки оригинальной кривой
        RGK::Vector<RGK::Math::Vector3D> controlPointsOriginalCurve = iCurve.GetControlPoints();
        int degree = iCurve.GetDegree();

        // Число кривых Безье, на которые будет разделена оригинальная кривая
        size_t numberBezierCurves = controlPointsOriginalCurve.size() / degree;
        RGK::Vector<RGK::NURBSCurve> bezierCurves(numberBezierCurves);
        RGK::NURBSCurve tempBezierCurve;

        RGK::Context rgkContext;
        RPLM::EP::Model::Session::GetSession()->GetRGKSession().CreateMainContext(rgkContext);

        for (size_t i = 0; i != numberBezierCurves; ++i)
        {
            RGK::Vector<RGK::Math::Vector3D> tempControlPoints;

            // Добавляем по частям контрольные точки оригинальной кривой для каждой отдельной кривой Безье
            for (size_t j = 0; j != static_cast<size_t>(degree) + 1; ++j)
            {
                tempControlPoints.push_back(controlPointsOriginalCurve[j + i * degree]);
            }

            RGK::NURBSCurve::CreateBezier(rgkContext, tempControlPoints, degree, tempBezierCurve);
            bezierCurves[i] = tempBezierCurve;
        }

        return bezierCurves;
    }

    int ConjugationMethod::CalcCombWithoutRepetition(int n, int k)
    {
        return k == 0 || k == n ? 1 : CalcCombWithoutRepetition(n - 1, k - 1) * n / k;
    }

    RGK::Math::Vector3D ConjugationMethod::calcDerivLeftBezierCurveForMerger(const RGK::Math::Vector3DArray& points, int currentIndex, int startIndex)
    {
        if (startIndex == currentIndex)
            return points[currentIndex];
        else
            return calcDerivLeftBezierCurveForMerger(points, currentIndex + 1, startIndex) - calcDerivLeftBezierCurveForMerger(points, currentIndex, startIndex - 1);
    }

    RGK::Math::Vector3D ConjugationMethod::calcNegativeDerivLeftBezierCurveForMerger(const RGK::Math::Vector3DArray& points, int currentIndex, int startIndex)
    {
        if (startIndex == currentIndex)
            return -1 * points[currentIndex];
        else
            return calcNegativeDerivLeftBezierCurveForMerger(points, currentIndex + 1, startIndex) - calcNegativeDerivLeftBezierCurveForMerger(points, currentIndex, startIndex - 1);
    }

    RGK::Math::Vector3D ConjugationMethod::calcDerivRightBezierCurveForMerger(const RGK::Math::Vector3DArray& points, int currentIndex, int startIndex)
    {
        if (startIndex == currentIndex)
            return points[currentIndex];
        else
            return calcDerivRightBezierCurveForMerger(points, currentIndex, startIndex + 1) - calcDerivRightBezierCurveForMerger(points, currentIndex - 1, startIndex);
    }

    void ConjugationMethod::FillCoefficientsMatrix(RGK::Vector<RGK::Vector<double>>& iCoefficientMatrix, RGK::Vector<RGK::Vector<double>>& iBasisFuncs, size_t iNumberEpsilons, size_t iNumberBreakPoints)
    {
        // Заполняем двойками главную диагональ
        for (size_t i = 0; i != iNumberEpsilons; ++i)
        {
            iCoefficientMatrix[i][i] = 2;
        }

        // Заполняет элементы матрицы коэффициентов над её главной диагональю
        auto FillUpperTriangularCoefficientMatrix = [&iCoefficientMatrix, &iBasisFuncs, iNumberEpsilons, iNumberBreakPoints]()  -> void
        {
            // Количество базисных функций
            const size_t NUMBER_BASIS_FUNCS = iBasisFuncs.size();

            // Каждый breakPoint - одна итерация заполнения базисных функций в coefficientMatrix
            for (size_t breakPointsCounter = 0; breakPointsCounter != iNumberBreakPoints; ++breakPointsCounter)
            {
                // Реверс строка для противоположной стороны треугольника
                size_t reverseRow = NUMBER_BASIS_FUNCS * 2 - 1 + NUMBER_BASIS_FUNCS * breakPointsCounter;
                size_t colBasisFunc = 0;

                // Идём по общему числу базисных функций
                for (size_t row = 0 + NUMBER_BASIS_FUNCS * breakPointsCounter; row != NUMBER_BASIS_FUNCS + NUMBER_BASIS_FUNCS * breakPointsCounter; ++row)
                {
                    // Строка базисных функций
                    size_t rowBasisFunc = 0;
                    // Предыдущее значение базисной функции (для правильного заполнения коэффициентов с нужным знаком "+" или "-")
                    double prevBasisFuncVal = iBasisFuncs[rowBasisFunc][colBasisFunc];

                    for (size_t col = iNumberEpsilons + NUMBER_BASIS_FUNCS * breakPointsCounter; col != iNumberEpsilons + NUMBER_BASIS_FUNCS + NUMBER_BASIS_FUNCS * breakPointsCounter; ++col)
                    {
                        double nextBasisFuncVal = iBasisFuncs[rowBasisFunc][colBasisFunc];
                        iCoefficientMatrix[row][col] = nextBasisFuncVal;

                        // Регулируем знак у противоположной части базисных функций
                        if (prevBasisFuncVal < 0 && nextBasisFuncVal < 0)
                        {
                            nextBasisFuncVal *= -1;
                        }
                        else if (prevBasisFuncVal >= 0 && nextBasisFuncVal > 0)
                        {
                            nextBasisFuncVal *= -1;
                        }

                        iCoefficientMatrix[reverseRow][col] = nextBasisFuncVal;
                        prevBasisFuncVal = nextBasisFuncVal;
                        ++rowBasisFunc;
                    }

                    --reverseRow;
                    ++colBasisFunc;
                }
            }
        };

        // Заполняет элементы матрицы коэффициентов под её главной диагональю
        auto FillLowerTriangularCoefficientMatrix = [&iCoefficientMatrix, &iBasisFuncs, iNumberEpsilons, iNumberBreakPoints]() -> void
        {
            // Количество базисных функций
            const size_t NUMBER_BASIS_FUNCS = iBasisFuncs.size();

            // Каждый breakPoint - одна итерация заполнения базисных функций в coefficientMatrix
            for (size_t breakPointCounter = 0; breakPointCounter != iNumberBreakPoints; ++breakPointCounter)
            {
                size_t rowBasisFunc = 0;

                // Итерируемся по общему числу базисных функций
                for (size_t row = iNumberEpsilons + NUMBER_BASIS_FUNCS * breakPointCounter; row != iNumberEpsilons + NUMBER_BASIS_FUNCS + NUMBER_BASIS_FUNCS * breakPointCounter; ++row)
                {
                    // Реверс столбец для противоположной части
                    size_t reverseCol = NUMBER_BASIS_FUNCS * 2 - 1 + NUMBER_BASIS_FUNCS * breakPointCounter;
                    size_t colBasisFunc = 0;
                    // Предыдущее значение базисной функции (для правильного заполнения коэффициентов с нужным знаком "+" или "-")
                    double prevBasisFuncVal = iBasisFuncs[rowBasisFunc][colBasisFunc];

                    for (size_t col = 0 + NUMBER_BASIS_FUNCS * breakPointCounter; col != NUMBER_BASIS_FUNCS + NUMBER_BASIS_FUNCS * breakPointCounter; ++col)
                    {
                        double nextBasisFuncVal = iBasisFuncs[rowBasisFunc][colBasisFunc];
                        iCoefficientMatrix[row][col] = nextBasisFuncVal;

                        // Регулируем знак у противоположной части базисных функций
                        if (prevBasisFuncVal < 0 && nextBasisFuncVal < 0 && col != NUMBER_BASIS_FUNCS * breakPointCounter)
                        {
                            nextBasisFuncVal *= -1;
                        }
                        else if (prevBasisFuncVal >= 0 && nextBasisFuncVal > 0)
                        {
                            nextBasisFuncVal *= -1;
                        }

                        prevBasisFuncVal = nextBasisFuncVal;
                        iCoefficientMatrix[row][reverseCol] = nextBasisFuncVal;
                        ++colBasisFunc;
                        --reverseCol;
                    }

                    ++rowBasisFunc;
                }
            }
        };

        // Заполняем матрицу коэффициентов базисными функциями
        FillUpperTriangularCoefficientMatrix();
        FillLowerTriangularCoefficientMatrix();
    }

    void ConjugationMethod::FixateCurve(RGK::Vector<RGK::Vector<double>>& iCoefficientMatrix, size_t iNumberEpsilons, size_t iNumberBasisFuncs, bool iFixBeginningCurve, bool iFixEndCurve)
    {
        int orderFixFirstDeriv = 1;
        int orderFixLastDeriv = 1;

        if (iFixBeginningCurve)
        {
            // Фиксация первой граничной точки кривой
            while (orderFixFirstDeriv >= 0)
            {
                for (size_t row = iNumberEpsilons; row != iNumberEpsilons + iNumberBasisFuncs; ++row)
                {
                    iCoefficientMatrix[orderFixFirstDeriv][row] = 0;
                }

                --orderFixFirstDeriv;
            }
        }

        if (iFixEndCurve)
        {
            for (size_t col = iNumberEpsilons + iNumberBasisFuncs; col != iCoefficientMatrix.size(); ++col)
            {
                iCoefficientMatrix[iNumberEpsilons - 1][col] = 0;
            }
        }


        // Не работает для 1 порядка производной
        //int tempCounter = 1;

        //while (orderFixLastDeriv >= 0)
        //{
        //    for (size_t col = numberEpsilons; col != numberEpsilons + numberBasisFuncs; ++col)
        //    {
        //        coefficientMatrix[numberEpsilons - tempCounter][col] = 0;
        //    }

        //    ++tempCounter;
        //    --orderFixLastDeriv;
        //}
    }

    void ConjugationMethod::FillFreeMemberMatrix(RGK::Vector<RGK::Vector<double>>& iFreeMembersMatrix, const RGK::Vector<RGK::Math::Vector3DArray>& iControlPointsBezierCurves, RGK::Vector<RGK::Vector<double>>& iBasisFuncs, RGK::Vector<RGK::Vector<double>>& iReverseBasisFuncs, size_t iNumberEpsilons)
    {
        size_t indexFreeMembers = iNumberEpsilons;

        for (size_t row = 0; row != iControlPointsBezierCurves.size() - 1; ++row)
        {
            size_t rowBasisFunc = 0;

            for (size_t col = 0; col != iBasisFuncs[0].size(); ++col)
            {
                for (size_t i = 0; i != iControlPointsBezierCurves[0].size(); ++i)
                {
                    // Текущая кривая
                    iFreeMembersMatrix[indexFreeMembers][0] += iControlPointsBezierCurves[row][i].GetX() * -iBasisFuncs[rowBasisFunc][i];
                    iFreeMembersMatrix[indexFreeMembers][1] += iControlPointsBezierCurves[row][i].GetY() * -iBasisFuncs[rowBasisFunc][i];
                    iFreeMembersMatrix[indexFreeMembers][2] += iControlPointsBezierCurves[row][i].GetZ() * -iBasisFuncs[rowBasisFunc][i];
                    // Следующая кривая
                    iFreeMembersMatrix[indexFreeMembers][0] += iControlPointsBezierCurves[row + 1][i].GetX() * iReverseBasisFuncs[rowBasisFunc][i];
                    iFreeMembersMatrix[indexFreeMembers][1] += iControlPointsBezierCurves[row + 1][i].GetY() * iReverseBasisFuncs[rowBasisFunc][i];
                    iFreeMembersMatrix[indexFreeMembers][2] += iControlPointsBezierCurves[row + 1][i].GetZ() * iReverseBasisFuncs[rowBasisFunc][i];
                }

                ++rowBasisFunc;
                ++indexFreeMembers;
            }
        }
    }
    RGK::Vector<RGK::Vector<double>> ConjugationMethod::CalculateShiftPoints(const RGK::Vector<RGK::Vector<double>>& iCoefficientMatrix, const RGK::Vector<RGK::Vector<double>>& iFreeMembersMatrix)
    {
        // Создаём указатель на интерфейс операций СЛАУ
        auto operation = IMatrixOperations::GetMatrixOperationsClass(OperationClass::eigen);

        if (operation == nullptr)
        {
            throw "Error! _calculateShiftPoints: operation = nullptr";
        }

        // Вычисляем определитель матрицы коэффициентов
        double coefficientMatrixDet = operation->GetMatrixDet(iCoefficientMatrix);

        if (coefficientMatrixDet == 0)
        {
            throw "Error! _calculateShiftPoints: Определитель матрицы коэффициентов = 0! Возможно, сделайте меньше фиксированных точек в функции fixPointsAtCurve!";
        }

        // Решаем СЛАУ
        return operation->SolveEquation(iCoefficientMatrix, iFreeMembersMatrix);
    }

    void ConjugationMethod::AdjustControlPoints(RGK::Vector<RGK::Math::Vector3DArray>& iControlPointsBezierCurves, RGK::Vector<RGK::Vector<double>>& iShiftPoints, size_t iNumberBezierCurves)
    {
        int tempCounter = 0;

        for (size_t i = 0; i != iNumberBezierCurves; ++i)
        {
            for (size_t j = 0; j != iControlPointsBezierCurves[i].size(); ++j)
            {
                double x = iControlPointsBezierCurves[i][j].GetX() + iShiftPoints[tempCounter][0];
                double y = iControlPointsBezierCurves[i][j].GetY() + iShiftPoints[tempCounter][1];
                double z = iControlPointsBezierCurves[i][j].GetZ() + iShiftPoints[tempCounter][2];
                iControlPointsBezierCurves[i][j].SetXYZ(x, y, z);
                ++tempCounter;
            }
        }
    }

    RGK::Vector<RGK::NURBSCurve> ConjugationMethod::CreateBezierCurves(RGK::Vector<RGK::Math::Vector3DArray>& iControlPointsBezierCurves, size_t iNumberBezierCurves, int iDegree)
    {
        RGK::Vector<RGK::NURBSCurve> newBezierCurves;
        RGK::NURBSCurve tempBezierCurve;

        for (size_t i = 0; i != iNumberBezierCurves; ++i)
        {
            // TODO! Не знаю, нужно ли каждый раз пересоздавать rgkContext и прописывать его в NURBSCurve::CreateBezier...
            RGK::Context rgkContext;
            RPLM::EP::Model::Session::GetSession()->GetRGKSession().CreateMainContext(rgkContext);

            // Создаём новую кривую Безье и добавляем в вектор, чтобы функция возвратила его
            RGK::NURBSCurve::CreateBezier(rgkContext, iControlPointsBezierCurves[i], iDegree, tempBezierCurve);
            newBezierCurves.push_back(tempBezierCurve);
        }

        return newBezierCurves;
    }

    RGK::NURBSCurve ConjugationMethod::BezierCurvesToNURBS(const RGK::Vector<RGK::NURBSCurve>& iBezierCurves, int iDegree)
    {
        RGK::Vector<RGK::Math::Vector3D> newControlPoints;
        // Для того, чтобы не было повторяющихся точек
        bool firstCheck = false;

        for (size_t curveCount = 0; curveCount != iBezierCurves.size(); ++curveCount)
        {
            RGK::Vector<RGK::Math::Vector3D> tempControlPoints = iBezierCurves[curveCount].GetControlPoints();

            for (size_t i = 0; i != tempControlPoints.size(); ++i)
            {
                if (firstCheck && i == 0)
                {
                    continue;
                }
                if (curveCount == 0)
                {
                    firstCheck = true;
                }

                newControlPoints.push_back(tempControlPoints[i]);
            }
        }
        // TODO! Не знаю, нужно ли каждый раз пересоздавать rgkContext и прописывать его в NURBSCurve::CreateBezier...
        RGK::Context rgkContext;
        RPLM::EP::Model::Session::GetSession()->GetRGKSession().CreateMainContext(rgkContext);

        Math::Geometry2D::Geometry::DoubleArray knots = NURBSUtils::FillEvenlyNodalVector(iDegree, static_cast<int>(newControlPoints.size()));

        RGK::NURBSCurve newCurve;
        // Создаём новую кривую Безье и добавляем в вектор, чтобы функция возвратила его
        RGK::NURBSCurve::Create(rgkContext, newControlPoints, iDegree, knots, false, newCurve);

        return newCurve;
    }
}