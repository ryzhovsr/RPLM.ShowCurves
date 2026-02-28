#pragma once

#include "Common/RPLM.Base.Framework.String.h"
#include "RPLM.GeomCore/RPLM.Math.Geometry2D/Geometry/RGPGeometryForward.h"

namespace RPLM::CAD
{
	class CurveParser
	{
	public:
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
		/// <returns>Массив кривых</returns>
		static std::vector<RGK::NURBSCurve> ReadCurvesFromFile(const Base::Framework::String& iFilePath);

		/// <summary>Сохраняет данные кривой в файл</summary>
		/// <param name="iCurve">Кривая</param>
		/// <param name="iFilePath">Путь к файлу</param>
		static void SaveCurveInFile(const RGK::NURBSCurve& iCurve, const Base::Framework::String& iFilePath);
	};
}
