#pragma once

#include <Geometry/Curves/NURBSCurve.h>
#include <Geometry/RGPGeometryForward.h>

namespace RPLM::CAD
{
	namespace ConjugationCurves
	{
		class ConjugationMethod
		{
		public:
			/// <summary>Сопряжение кривой, состоящей из множества кривых Безье</summary>
			/// <param name="iCurve">Кривая</param>
			/// <param name="iFixBeginningCurve">Зафиксировать начало кривой</param>
			/// <param name="iFixEndCurve">Зафиксировать конец кривой</param>
			/// <returns>Сопряжённая кривая</returns>
			RGK::NURBSCurve ConjugateCurve(const RGK::NURBSCurve& iCurve, bool iFixBeginningCurve, bool iFixEndCurve);

			/// <summary>Сопрягает 2 кривых Безье</summary>
			/// <param name="iCurve1">Первая кривая Безье</param>
			/// <param name="iCurve2">Вторая кривая Безье</param>
			/// <returns>Сопряжённая кривая</returns>
			RGK::NURBSCurve ConjugateCurves(const RGK::NURBSCurve& iCurve1, const RGK::NURBSCurve& iCurve2, bool iFixBeginningCurve = false, bool iFixEndCurve = false);

		private:
			/// <summary>Разбивает NURBS кривую на кривые Безье</summary>
			/// <param name="iCurve">Оригинальная кривая</param>
			/// <returns>Вектор кривых Безье</returns>
			RGK::Vector<RGK::NURBSCurve> DivideCurveIntoBezierCurves(const RGK::NURBSCurve& iCurve);

			int CalcCombWithoutRepetition(int n, int k);

			RGK::Math::Vector3D calcDerivLeftBezierCurveForMerger(const RGK::Math::Vector3DArray& points, int currentIndex, int startIndex);

			RGK::Math::Vector3D calcNegativeDerivLeftBezierCurveForMerger(const RGK::Math::Vector3DArray& points, int currentIndex, int startIndex);

			RGK::Math::Vector3D calcDerivRightBezierCurveForMerger(const RGK::Math::Vector3DArray& points, int currentIndex, int startIndex);

			/// <summary>Заполнение матрицы коэффициентов</summary>
			/// <param name="iCoefficientMatrix">Матрица коэффициентов</param>
			/// <param name="iBasisFuncs">Базисные функции</param>
			/// <param name="iNumberEpsilons">Количество эпсилон</param>
			/// <param name="iNumberBreakPoints">Число точек останова</param>
			void FillCoefficientsMatrix(RGK::Vector<RGK::Vector<double>>& iCoefficientMatrix, RGK::Vector<RGK::Vector<double>>& iBasisFuncs, size_t iNumberEpsilons, size_t iNumberBreakPoints);

			/// <summary>Фиксирует начало и конец кривой (зануление определенных столбцов и строк у матрицы коэффициентов)</summary>
			/// <param name="iCoefficientMatrix">Матрица коэффициентов</param>
			/// <param name="iNumberEpsilons">Количество эпсилон</param>
			/// <param name="iNumberBasisFuncs">Количество базисных функций</param>
			/// <param name="iFixBeginningCurve">Зафиксировать начало кривой</param>
			/// <param name="iFixEndCurve">Зафиксировать конец кривой</param>
			void FixateCurve(RGK::Vector<RGK::Vector<double>>& iCoefficientMatrix, size_t iNumberEpsilons, size_t iNumberBasisFuncs, bool iFixBeginningCurve, bool iFixEndCurve);

			/// <summary>Заполняет матрицу свободных членов</summary>
			/// <param name="iFreeMembersMatrix">Матрица свободных членов</param>
			/// <param name="iControlPointsBezierCurves">Контрольные точки кривых Безье</param>
			/// <param name="iBasisFuncs">Базисные функции</param>
			/// <param name="iReverseBasisFuncs">Обратные базисные функции</param>
			/// <param name="iNumberEpsilons">Количество эпсилон</param>
			void FillFreeMemberMatrix(RGK::Vector<RGK::Vector<double>>& iFreeMembersMatrix, const RGK::Vector<RGK::Math::Vector3DArray>& iControlPointsBezierCurves, RGK::Vector<RGK::Vector<double>>& iBasisFuncs, RGK::Vector<RGK::Vector<double>>& iReverseBasisFuncs, size_t iNumberEpsilons);

			/// <summary>Вычисляет точки сдвига для полного сопряжения кривой</summary>
			/// <param name="iCoefficientMatrix">Матрица коэффициентов</param>
			/// <param name="iFreeMembersMatrix">Матрица свободных членов</param>
			/// <returns>Точки сдвига</returns>
			RGK::Vector<RGK::Vector<double>> CalculateShiftPoints(const RGK::Vector<RGK::Vector<double>>& iCoefficientMatrix, const RGK::Vector<RGK::Vector<double>>& iFreeMembersMatrix);

			/// <summary>Регулирует контрольные точки кривых Безье</summary>
			/// <param name="iControlPointsBezierCurves">Контрольные точки кривых Безье</param>
			/// <param name="iShiftPoints">Точки сдвига</param>
			/// <param name="iNumberBezierCurves">Количество контрольных точек</param>
			void AdjustControlPoints(RGK::Vector<RGK::Math::Vector3DArray>& iControlPointsBezierCurves, RGK::Vector<RGK::Vector<double>>& iShiftPoints, size_t iNumberBezierCurves);
			
			/// <summary>Создаёт вектор кривых Безье из заданного вектора контрольных многоугольников кривых Безье</summary>
			/// <param name="iControlPointsBezierCurves">Контрольные точки кривых Безье</param>
			/// <param name="iNumberBezierCurves">Количество кривых Безье</param>
			/// <param name="iDegree">Степень</param>
			/// <returns>Вектор кривых Безье</returns>
			RGK::Vector<RGK::NURBSCurve> CreateBezierCurves(RGK::Vector<RGK::Math::Vector3DArray>& iControlPointsBezierCurves, size_t iNumberBezierCurves, int iDegree);

			/// <summary>Переводит вектор кривых Безье в один NURBS</summary>
			/// <param name="iBezierCurves">Вектор кривых Безье</param>
			/// <param name="iDegree">Степень</param>
			/// <returns>Сопряжённая кривая</returns>
			RGK::NURBSCurve BezierCurvesToNURBS(const RGK::Vector<RGK::NURBSCurve>& iBezierCurves, int iDegree);
		};
	}
}
