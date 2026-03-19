#pragma once

namespace RPLM::CAD::UI
{
	const RPLM::Base::Framework::String& GetModuleName();
}

#define RSCADUIW(key)	RPLM::Base::Framework::GetModuleResource(L##key, RPLM::CAD::UI::GetModuleName())
