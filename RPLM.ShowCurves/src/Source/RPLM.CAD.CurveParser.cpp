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
            throw std::runtime_error("Путь к файлу является пустой строкой.");

        std::ifstream inStream(iFilePath.c_str());

        if (!inStream.is_open())
            throw std::runtime_error("Не удалось открыть файл.");

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
        
        // Переменная отвечающая за строку в которой ошибка
        size_t lineNumber = 0;

        // Набор ожидаемых блоков для построения кривой
        const std::vector<std::string> expectedBlocks = {
            "Degree:", "IsPeriodic:", "Control Points[", "Weights[", "Knots["
        };

        while (std::getline(inStream, line))
        {
            // Увеличиваем счетчик строк
            lineNumber++;

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
            
            // Проверяем начинается ли кривая с ожидаемого блока
            bool startsWithExpectedBlock = false;
            for (const auto& block : expectedBlocks)
            {
                if (line.rfind(block, 0) == 0)
                {
                    startsWithExpectedBlock = true;
                    break;
                }
            }

            if (!startsWithExpectedBlock)
            {
                throw std::runtime_error("Неизвестный блок или опечатка (строка " + std::to_string(lineNumber) + ").");
            }

            // Степень
            if (line.rfind("Degree:", 0) == 0)
            {
                // Если уже есть данные предыдущей кривой, создаём её
                createCurveIfComplete();

                try {
                    degree = std::stoi(line.substr(line.find(':') + 1));
                }
                catch (...) {
                    throw std::runtime_error("Неверный формат Degree кривой (строка " + std::to_string(lineNumber) + ").");
                }

                if (degree < 0)
                    throw std::runtime_error("Degree не может иметь отрицательного значения (строка " + std::to_string(lineNumber) + ").");

                hasDegree = true;
                continue;
            }

            // Периодичность
            if (line.rfind("IsPeriodic:", 0) == 0)
            {
                std::string value = line.substr(line.find(':') + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);

                if (value == "true" || value == "1")
                    isPeriodic = true;
                else if (value == "false" || value == "0")
                    isPeriodic = false;
                else
                    throw std::runtime_error("Некорректное значение IsPeriodic, допустимо только true/false/1/0 (строка " + std::to_string(lineNumber) + ").");

                hasPeriodic = true;
                continue;
            }

            // Контрольные точки
            if (line.rfind("Control Points[", 0) == 0)
            {
                size_t lineNumberControlPointsDeclaration = lineNumber;

                auto l = line.find('[');
                auto r = line.find(']');

                if (l == std::string::npos || r == std::string::npos)
                    throw std::runtime_error("Некорректный формат блока Control Points (строка " + std::to_string(lineNumber) + ").");

                try {
                    expectedControlPoints = std::stoi(line.substr(l + 1, r - l - 1));
                }
                catch (...) {
                    throw std::runtime_error("Некорректное количество контрольных точек (строка " + std::to_string(lineNumber) + ").");
                }

                controlPoints.clear();
                controlPoints.reserve(expectedControlPoints);

                while (true)
                {
                    //Запоминаем текущую позицию в файле перед чтением строки
                    std::streampos posBeforeLine = inStream.tellg();

                    if (!std::getline(inStream, line)) break;

                    line.erase(0, line.find_first_not_of(" \t\r\n"));
                    line.erase(line.find_last_not_of(" \t\r\n") + 1);
                    
                    // Пропускаем пустые строки, если они есть, между точками
                    if (line.empty()) {
                        lineNumber++;
                        continue;
                    }
                    
                    bool isHeader = false;
                    for (const auto& block : expectedBlocks) {
                        if (line.rfind(block, 0) == 0) {
                            isHeader = true; break;
                        }
                    }
                    
                    if (isHeader) {
                        // Откатываемся на строку назад при нахождении следующего блока,
                        // поскольку внешний цикл перекинет нас на строку вперед
                        inStream.seekg(posBeforeLine);
                        break;
                    }

                    lineNumber++;

                    std::replace(line.begin(), line.end(), ',', ' ');
                    std::istringstream iss(line);

                    double x, y, z;

                    if (!(iss >> x >> y >> z))
                        throw std::runtime_error("Ошибка чтения координат контрольной точки (строка " + std::to_string(lineNumber) + ").");

                    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                        throw std::runtime_error("Некорректные координаты (NaN или inf) (строка " + std::to_string(lineNumber) + ").");

                    controlPoints.push_back(RGK::Math::Vector3D(x, y, z));
                }

                // Проверяем количество точек после окончания блока
                if (controlPoints.size() != static_cast<size_t>(expectedControlPoints))
                    throw std::runtime_error(
                        "Количество контрольных точек не совпадает с ожидаемым (строка " +
                        std::to_string(lineNumberControlPointsDeclaration) + ")."
                    );

                hasControlPoints = true;
                continue;
            }

            // Весовые коэффициенты
            if (line.rfind("Weights[", 0) == 0)
            {
                // Номер строки Weights;
                // нужен для того, чтобы в случае несоответствия ожидаемых и фактических точек в тексте ошибки указывалась корректная строка
                size_t lineNumberWeightsDeclaration = lineNumber;

                auto l = line.find('[');
                auto r = line.find(']');

                if (l == std::string::npos || r == std::string::npos)
                    throw std::runtime_error("Некорректный формат блока Weights (строка " + std::to_string(lineNumber) + ").");

                try {
                expectedWeights = std::stoi(line.substr(l + 1, r - l - 1));
                }
                catch (...) {
                    throw std::runtime_error("Некорректный формат количества весовых коэффициентов кривой (строка " + std::to_string(lineNumber) + ").");
                }

                weights.reserve(expectedWeights);

                while (weights.size() < static_cast<size_t>(expectedWeights) && std::getline(inStream, line))
                {
                    std::replace(line.begin(), line.end(), ',', ' ');
                    std::istringstream iss(line);
                    double w;

                    // Увеличиваем счетчик строк
                    lineNumber++;

                    while (iss >> w)
                    {
                        if (!std::isfinite(w))
                            throw std::runtime_error("Некорректный вес (NaN или inf) (строка " + std::to_string(lineNumber) + ").");
                        weights.push_back(w);
                    }

                    if (!iss.eof() && iss.fail()) {
                        throw std::runtime_error("Ошибка формата данных Weights: встречено не числовое значение (строка " + std::to_string(lineNumber) + ").");
                    }
                }

                if (weights.size() != static_cast<size_t>(expectedWeights))
                    throw std::runtime_error("Количество считанных весов не совпадает с ожидаемым (строка " + std::to_string(lineNumberWeightsDeclaration) + ").");

                hasWeights = true;

                // После весов может начинаться следующая кривая
                continue;
            }

            // Узловые коэффициенты
            if (line.rfind("Knots[", 0) == 0)
            {
                // Номер строки Knots; нужен для того, чтобы в случае несоответствия ожидаемых и фактических точек в тексте ошибки указывалась корректная строка
                size_t lineNumberKnotsDeclaration = lineNumber;

                auto l = line.find('[');
                auto r = line.find(']');

                if (l == std::string::npos || r == std::string::npos || l >= r)
                    throw std::runtime_error("Некорректный формат блока Knots (строка " + std::to_string(lineNumber) + ").");

                try {
                    expectedKnots = std::stoi(line.substr(l + 1, r - l - 1));
                }
                catch (...)
                {
                    throw std::runtime_error("Некорректное количество узлов (строка " + std::to_string(lineNumber) + ").");
                }

                knots.reserve(expectedKnots);

                while (knots.size() < static_cast<size_t>(expectedKnots) && std::getline(inStream, line))
                {
                    std::replace(line.begin(), line.end(), ',', ' ');
                    std::istringstream iss(line);
                    double k;

                    // Увеличиваем счетчик строк
                    lineNumber++;

                    while (iss >> k)
                    {
                        if (!std::isfinite(k))
                            throw std::runtime_error("Некорректный узел (NaN или inf) (строка " + std::to_string(lineNumber) + ").");
                        knots.push_back(k);
                    }

                    if (!iss.eof() && iss.fail()) {
                        throw std::runtime_error("Ошибка формата данных Knots: встречено не числовое значение (строка " + std::to_string(lineNumber) + ").");
                    }
                }

                if (knots.size() != static_cast<size_t>(expectedKnots))
                    throw std::runtime_error("Количество считанных узлов не совпадает с ожидаемым (строка " + std::to_string(lineNumberKnotsDeclaration) + ").");

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
