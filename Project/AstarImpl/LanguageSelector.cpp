#include "LanguageSelector.h"

namespace mawile {
	std::string LScout(mawile::LanguageSelector _LanguageSelector, std::string _en, std::string _jp, std::string _ko) {
		if (_LanguageSelector == mawile::LanguageSelector::English) {
			std::cout << _en;

			return _en;
		}
		else if (_LanguageSelector == mawile::LanguageSelector::Japanese) {
			std::cout << _jp;

			return _jp;
		}
		else if (_LanguageSelector == mawile::LanguageSelector::Korean) {
			std::cout << _ko;

			return _ko;
		}

		return _en;
	}

	std::wstring LSwcout(mawile::LanguageSelector _LanguageSelector, std::wstring _en, std::wstring _jp, std::wstring _ko) {
		_setmode(_fileno(stdout), _O_U8TEXT);

		if (_LanguageSelector == mawile::LanguageSelector::English) {
			std::wcout << _en;
			_setmode(_fileno(stdout), _O_TEXT);

			return _en;
		}
		else if (_LanguageSelector == mawile::LanguageSelector::Japanese) {
			std::wcout << _jp;
			_setmode(_fileno(stdout), _O_TEXT);

			return _jp;
		}
		else if (_LanguageSelector == mawile::LanguageSelector::Korean) {
			std::wcout << _ko;
			_setmode(_fileno(stdout), _O_TEXT);

			return _ko;
		}

		return _en;
	}

	std::string LSstream(std::ofstream& ofs, mawile::LanguageSelector _LanguageSelector, std::string _en, std::string _jp, std::string _ko) {
		if (_LanguageSelector == mawile::LanguageSelector::English) {
			ofs << _en;

			return _en;
		}
		else if (_LanguageSelector == mawile::LanguageSelector::Japanese) {
			ofs << _jp;

			return _jp;
		}
		else if (_LanguageSelector == mawile::LanguageSelector::Korean) {
			ofs << _ko;

			return _ko;
		}

		return _en;
	}

	std::wstring LSwstream(std::wofstream ofs, mawile::LanguageSelector _LanguageSelector, std::wstring _en, std::wstring _jp, std::wstring _ko) {
		std::locale::global(std::locale(".ANSI"));

		if (_LanguageSelector == mawile::LanguageSelector::English) {
			ofs << _en;

			return _en;
		}
		else if (_LanguageSelector == mawile::LanguageSelector::Japanese) {
			ofs << _jp;

			return _jp;
		}
		else if (_LanguageSelector == mawile::LanguageSelector::Korean) {
			ofs << _ko;

			return _ko;
		}

		return _en;
	}
}