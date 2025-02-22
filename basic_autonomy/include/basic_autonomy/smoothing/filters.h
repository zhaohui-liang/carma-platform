#pragma once

/*
 * Copyright (C) 2019-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <vector>
#include <deque>
#include <algorithm>
#include <stdexcept>
namespace basic_autonomy
{
namespace smoothing
{


/**
 * \brief Extremely simplie moving average filter
 * 
 * \param input The points to be filtered
 * \param window_size The number of points to use in the moving window for averaging
 * 
 * \return The filterted points
 */
std::vector<double> moving_average_filter(const std::vector<double> input, int window_size, bool ignore_first_point=true);

};  // namespace smoothing
};  // namespace basic_autonomy