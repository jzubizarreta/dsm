/**
* This file is part of DSM.
*
* Copyright (C) 2019 CEIT (Universidad de Navarra) and Universidad de Zaragoza
* Developed by Jon Zubizarreta,
* for more information see <https://github.com/jzubizarreta/dsm>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "DSMLib.h"
#include <iostream>

bool printDSMLibVersion()
{
	std::cout << "DSM: Direct Sparse Mapping\n" << 
				 "Copyright (C) 2019 CEIT (Universidad de Navarra) and Universidad de Zaragoza\n" << 
				 "Developed by Jon Zubizarreta,\n" << 
				 "for more information see <https://github.com/jzubizarreta/dsm>.\n" <<
				 "If you use this code, please cite the respective publications as\n" << 
				 "listed on the above website.\n" << std::endl;
	return true;
}

static bool init = printDSMLibVersion();