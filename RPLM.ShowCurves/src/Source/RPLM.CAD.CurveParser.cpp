#include "RPLM.CAD.CurveParser.h"
#include "Geometry/Curves/NURBSCurve.h"
#include "RGPSession.h"
#include <fstream>

namespace RPLM::CAD
{
	RGK::Vector<RGK::Math::Vector3D> CurveParser::ReadControlPointsFromFile(const Base::Framework::String& iFilePath)
	{
		std::ifstream inStream(iFilePath);
		RGK::Vector<RGK::Math::Vector3D> controlPoints;

		// Счётчик координат
		int coordinateCounter = 0;
		// Размерность координат
		const int coordinateDimension = 3;

		if (inStream.is_open())
		{
			RGK::Vector<double> temp;
			double number = 0;

			while (inStream >> number)
			{
				// Если все три координаты получены
				if (coordinateCounter == coordinateDimension)
				{
					controlPoints.push_back(RGK::Math::Vector3D(temp[0], temp[1], temp[2]));
					temp.clear();
					coordinateCounter = 0;
				}

				temp.push_back(number);
				++coordinateCounter;
			}

			// Добавляем последнюю координату
			if (coordinateCounter == coordinateDimension)
			{
				controlPoints.push_back(RGK::Math::Vector3D(temp[0], temp[1], temp[2]));
			}

			inStream.close();
		}

		return controlPoints;
	}

	Math::Geometry2D::Geometry::DoubleArray CurveParser::ReadKnotsFromFile(const Base::Framework::String& iFilePath)
	{
		Math::Geometry2D::Geometry::DoubleArray knots;
		std::ifstream inStream(iFilePath);

		if (inStream.is_open())
		{
			double number = 0;

			while (inStream >> number)
			{
				knots.push_back(number);
			}

			inStream.close();
		}

		return knots;
	}

    std::vector<RGK::NURBSCurve> CurveParser::ReadCurvesFromFile(const Base::Framework::String& iFilePath)
    {
        std::vector<RGK::NURBSCurve> curves;

        if (iFilePath.empty())
            return curves;

        std::ifstream inStream(iFilePath.c_str());

        if (!inStream.is_open())
            return curves;

        RGK::Context rgkContext;
        EP::Model::Session::GetSession()->GetRGKSession().CreateMainContext(rgkContext);

        std::string line;

        // Переменные для текущей считываемой кривой
        int degree = 0;
        bool isPeriodic = false;
        RGK::Math::Vector3DArray controlPoints;
        std::vector<double> weights;
        std::vector<double> knots;

        // Ожидаемые количества элементов
        int expectedControlPoints = -1;
        int expectedWeights = -1;
        int expectedKnots = -1;

        // Счётчики для проверки, когда все данные для кривой собраны
        bool hasDegree = false;
        bool hasPeriodic = false;
        bool hasControlPoints = false;
        bool hasWeights = false;
        bool hasKnots = false;

        // Вспомогательная функция для создания кривой из собранных данных
        auto createCurveIfComplete = [&]() -> bool
        {
            if (hasDegree && hasPeriodic && hasControlPoints && hasWeights && hasKnots)
            {
                // Проверяем согласованность данных
                if (controlPoints.size() != static_cast<size_t>(expectedControlPoints) ||
                    weights.size() != static_cast<size_t>(expectedWeights) ||
                    knots.size() != static_cast<size_t>(expectedKnots))
                {
                    return false;
                }

                // Создаём NURBS кривую
                RGK::NURBSCurve curve;

                if (RGK::NURBSCurve::Create(rgkContext, controlPoints, degree, knots, isPeriodic, curve) == RGK::Result::Success)
                {
                    curves.push_back(curve);

                    // Сбрасываем данные для следующей кривой
                    degree = 0;
                    isPeriodic = false;
                    controlPoints.clear();
                    weights.clear();
                    knots.clear();
                    expectedControlPoints = -1;
                    expectedWeights = -1;
                    expectedKnots = -1;
                    hasDegree = false;
                    hasPeriodic = false;
                    hasControlPoints = false;
                    hasWeights = false;
                    hasKnots = false;

                    return true;
                }
            }

            return false;
        };

        while (std::getline(inStream, line))
        {
            // Удаляем все пробелы, табуляции и переводы строк слева и справа
            line.erase(0, line.find_first_not_of(" \t\r\n"));
            line.erase(line.find_last_not_of(" \t\r\n") + 1);

            // Если пустая строка, проверяем, не завершилась ли кривая
            if (line.empty())
            {
                // Пустая строка может быть разделителем между кривыми
                createCurveIfComplete();
                continue;
            }

            // Степень
            if (line.rfind("Degree:", 0) == 0)
            {
                // Если уже есть данные предыдущей кривой, создаём её
                createCurveIfComplete();

                degree = std::stoi(line.substr(line.find(':') + 1));
                hasDegree = true;
                continue;
            }

            // Периодичность
            if (line.rfind("IsPeriodic:", 0) == 0)
            {
                std::string value = line.substr(line.find(':') + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                isPeriodic = (value == "true" || value == "1");
                hasPeriodic = true;
                continue;
            }

            // Контрольные точки
            if (line.rfind("Control Points[", 0) == 0)
            {
                auto l = line.find('[');
                auto r = line.find(']');
                expectedControlPoints = std::stoi(line.substr(l + 1, r - l - 1));
                controlPoints.reserve(expectedControlPoints);

                for (int i = 0; i < expectedControlPoints; ++i)
                {
                    // Ошибка чтения
                    if (!std::getline(inStream, line))
                        return curves;

                    std::replace(line.begin(), line.end(), ',', ' ');
                    std::istringstream iss(line);
                    double x, y, z;

                    if (iss >> x >> y >> z)
                    {
                        controlPoints.push_back(RGK::Math::Vector3D(x, y, z));
                    }
                    else
                    {
                        // Ошибка формата
                        return curves;
                    }
                }

                hasControlPoints = true;
                continue;
            }

            // Весовые коэффициенты
            if (line.rfind("Weights[", 0) == 0)
            {
                auto l = line.find('[');
                auto r = line.find(']');
                expectedWeights = std::stoi(line.substr(l + 1, r - l - 1));
                weights.reserve(expectedWeights);

                while (weights.size() < static_cast<size_t>(expectedWeights) && std::getline(inStream, line))
                {
                    std::replace(line.begin(), line.end(), ',', ' ');
                    std::istringstream iss(line);
                    double w;

                    while (iss >> w)
                        weights.push_back(w);
                }

                if (weights.size() != static_cast<size_t>(expectedWeights))
                    return curves; // Ошибка: не все веса считаны

                hasWeights = true;

                // После весов может начинаться следующая кривая
                continue;
            }

            // Узловые коэффициенты
            if (line.rfind("Knots[", 0) == 0)
            {
                auto l = line.find('[');
                auto r = line.find(']');
                expectedKnots = std::stoi(line.substr(l + 1, r - l - 1));
                knots.reserve(expectedKnots);

                while (knots.size() < static_cast<size_t>(expectedKnots) && std::getline(inStream, line))
                {
                    std::replace(line.begin(), line.end(), ',', ' ');
                    std::istringstream iss(line);
                    double k;

                    while (iss >> k)
                        knots.push_back(k);
                }

                // Ошибка: не все узлы считаны
                if (knots.size() != static_cast<size_t>(expectedKnots))
                    return curves;

                hasKnots = true;

                // После узлов кривая полностью описана
                continue;
            }
        }

        // Проверяем, не осталась ли последняя кривая необработанной
        createCurveIfComplete();

        return curves;
    }

	void CurveParser::SaveCurveInFile(const RGK::NURBSCurve& iCurve, const Base::Framework::String& iFilePath)
	{
		if (!iCurve || iFilePath.empty())
			return;

		std::ofstream outStream(iFilePath);

		if (outStream.is_open())
		{
			outStream << "Degree: " << iCurve.GetDegree() << "\n";
			outStream << "IsPeriodic: " << iCurve.IsPeriodic() << "\n";

			const auto& controlPoints = iCurve.GetControlPoints();
			outStream << "Control Points[" << controlPoints.size() << "]:\n";

			for (const auto& point : controlPoints)
				outStream << point.GetX() << ", " << point.GetY() << ", " << point.GetZ() << '\n';

			// Выводит одномерный массив в файл по заготовленному шаблону
			const auto& SaveOneDimensionalArrayInFile = [&outStream](const Math::Geometry2D::Geometry::DoubleArray& iArray)
			{
				size_t arraySize = iArray.size();

				for (size_t i = 0; i != arraySize; ++i)
				{
					outStream << iArray[i];

					if (i != arraySize - 1)
					{
						outStream << ", ";
					}
				}

				outStream << "\n";
			};

			const auto& weights = iCurve.GetWeights();
			outStream << "Weights[" << weights.size() << "]:\n";
			SaveOneDimensionalArrayInFile(weights);

			const auto& knots = iCurve.GetKnots();
			outStream << "Knots[" << knots.size() << "]:\n";
			SaveOneDimensionalArrayInFile(knots);

			outStream.close();
		}
	}
}
