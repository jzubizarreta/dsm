# 
# This file is part of DSM.
# 
# Copyright (C) 2019 CEIT (Universidad de Navarra) and Universidad de Zaragoza
# Developed by Jon Zubizarreta <jzubizarreta at ceit dot es>,
# for more information see <https://github.com/jzubizarreta/dsm>.
# If you use this code, please cite the respective publications as
# listed on the above website.
# 
# DSM is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# DSM is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with DSM. If not, see <http://www.gnu.org/licenses/>.
# 

# flag to set see instructions
option(ENABLE_SSE "Use Intel SSE instrunctions" ON)

# create configuration file
configure_file(	${CMAKE_CURRENT_LIST_DIR}/BuildFlags.h.in 
				BuildFlags.h 
				@ONLY)