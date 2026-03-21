#pragma once

#include "Common/RPLM.Base.Framework.String.h"
#include "RPLM.GeomCore/RPLM.Math.Geometry2D/Geometry/RGPGeometryForward.h"

namespace RPLM::CAD
{
	class CurveParser
	{
	public:
		/// <summary>
		/// Результаты выполнения операции парсинга файла с кривыми.
		/// Содержит коды ошибок валидации структуры, данных и успешного завершения.
		/// </summary>
		enum class ParserResult {
			// Ошибка открытия файла
			ErrorOpenFile = 1,

			// Неизвестный/Неожиданный блок при чтение файла
			UnexpectedBlock = 2,

			// Неверный формат блока Degree 
			IncorrectFormatBlockDegree = 3,

			// Параметр Degree не может быть отрицательным
			DegreeParameterCannotBeNegative = 4,

			// Параметр IsPeriodic некорректный
			IsPeriodicParameterIncorrect = 5,

			// Неверный формат блока ControlPoints
			IncorrectFormatBlockControlPoints = 6,

			// Некорректное количество ожидаемых контрольных точек
			IncorrectNumberExpectControlPoints = 7,

			// Ошибка чтения координаты контрольной точки
			ErrorCoordinateControlPoint = 8,

			// Количество фактических контрольных точек не совпадает с количеством ожидаемых
			NumberActualControlPointsNotMatchExpect = 9,

			// Неверный формат блока Degree 
			IncorrectFormatBlockWeights = 10,

			// Некорректное количество ожидаемых весовых коэффициентов
			IncorrectNumberExpectWeights = 11,

			// Ошибка чтения весового коэффициента
			ErrorWeight = 12,

			// Количество фактических весовых коэффициентов не совпадает с количеством ожидаемых
			NumberActualWeightsNotMatchExpect = 13,

			// Неверный формат блока Knots 
			IncorrectFormatBlockKnots = 14,

			// Некорректное количество ожидаемых узловых коэффициентов
			IncorrectNumberExpectKnots = 15,

			// Ошибка чтения узлового коэффициента
			ErrorKnot = 16,

			// Количество фактических узловых коэффициентов не совпадает с количеством ожидаемых
			NumberActualKnotsNotMatchExpect = 17,

			// Успешное чтение файла
			SuccessReadFile = 18,
		};
		
		/// <summary>
		/// Расширенная информация об ошибке парсинга файла.
		/// Позволяет связать код результата с конкретным местом в исходном файле.
		/// </summary>
		struct ParseError {
			/// <summary> Код результата (ошибки или успеха). </summary>
			ParserResult codeError;
			/// <summary> Номер строки в файле, на которой возникла проблема. Значение -1, если не применимо. </summary>
			int lineNumber = -1;
		};

		/// <summary>Считывает контрольные точки кривой из файла</summary>
		/// <param name="iFilePath">Путь к файлу</param>
		/// <returns>Контрольные точки</returns>
		static RGK::Vector<RGK::Math::Vector3D> ReadControlPointsFromFile(const Base::Framework::String& iFilePath);

		/// <summary>Считывает узловой вектор из файла</summary>
		/// <param name="iFilePath">Путь к файлу</param>
		/// <returns>Узловой вектор</returns>
		static Math::Geometry2D::Geometry::DoubleArray ReadKnotsFromFile(const Base::Framework::String& iFilePath);

		/// <summary>Считывает кривые из файла</summary>
		/// <param name="iFilePath">Путь к файлу</param>
		/// <returns>Результат чтения из файла</returns>
		static ParseError ReadCurvesFromFile(const Base::Framework::String& iFilePath, std::vector<RGK::NURBSCurve>& oCurves);

		/// <summary>Сохраняет данные кривой в файл</summary>
		/// <param name="iCurve">Кривая</param>
		/// <param name="iFilePath">Путь к файлу</param>
		static void SaveCurveInFile(const RGK::NURBSCurve& iCurve, const Base::Framework::String& iFilePath);
	};
}
