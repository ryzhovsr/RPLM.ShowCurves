#pragma once

#include "Common/RPLM.Base.Framework.String.h"

namespace RPLM::CAD
{
	namespace UI
	{
		class Session
		{
		public:
			Session(const Session&) = delete;
			Session& operator=(const Session&) = delete;
			Session(Session&&) = delete;
			Session& operator=(Session&&) = delete;

			static Session& Instance();

			// Инициализация
			void Init();

			// Завершение
			void Destroy();

			/// <summary>Возвращает название модуля</summary>
			/// <returns>Название модуля</returns>
			const RPLM::Base::Framework::String& GetModuleName();

		private:
			Session();
			~Session();
		};
	}
}
