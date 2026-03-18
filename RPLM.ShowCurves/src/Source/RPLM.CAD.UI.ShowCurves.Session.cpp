#include "RPLM.CAD.UI.ShowCurves.Session.h"
#include "RPLM.Base.Framework/Common/RPLM.Base.Framework.ResourceDefs.h"

namespace RPLM::CAD
{
	namespace UI
	{
		Session::Session()
		{
		}

		Session::~Session()
		{
			Destroy();
		}

		void Session::Init()
		{
			RPLM::Base::Framework::LoadResourcesForCurrentModule(GetModuleName().c_str());
		}

		void Session::Destroy()
		{
		}

		Session& Session::Instance()
		{
			static Session session;
			return session;
		}

		const RPLM::Base::Framework::String& Session::GetModuleName()
		{
			static RPLM::Base::Framework::String _name(L"RPLM.CAD.ShowCurves");
			return _name;
		}

		const RPLM::Base::Framework::String& GetModuleName()
		{
			return Session::Instance().GetModuleName();
		}
	}
}