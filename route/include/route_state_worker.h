#pragma once

/*
 * Copyright (C) 2020-2021 LEIDOS.
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

#include <stdexcept>

namespace route {

    class RouteStateWorker {

        public:

            /**
            * Possible events for the RouteStateWorker to respond to:
            * ROUTE_LOADED - Route worker received all necessary parameters and loaded route file path
            * ROUTE_SELECTED - A route file has been selected by user
            * ROUTE_STARTED - A route is generated by provided route file
            * ROUTE_COMPLETED - Reached the destination of current route
            * ROUTE_DEPARTED - Vehicle has deviated from the route
            * ROUTE_ABORTED - User decides to stop following the current route
            * ROUTE_GEN_FAILED - No route can be generated based on provided route file
            * ROUTE_INVALIDATION - Certain portion of the route is invalidated based on situation.
            */
            enum RouteEvent {
                ROUTE_LOADED = 0,
                ROUTE_SELECTED = 1,
                ROUTE_STARTED = 2,
                ROUTE_COMPLETED = 3,
                ROUTE_DEPARTED = 4,
                ROUTE_ABORTED = 5,
                ROUTE_GEN_FAILED = 6,
                ROUTE_INVALIDATION = 7,
            };

            /**
            * Possible states of the RouteStateWorker:
            * LOADING - RouteState worker's initial state, waiting for all necessary parameters to be set
            * SELECTION - RouteState worker is waiting on user to select a route
            * ROUTING - Calling lanelet2 library to generate a route based on selected route file
            * FOLLOWING - Following a route and tracking its downtrack and crosstrack distance
            */
            enum RouteState {
                LOADING = 0,
                SELECTION = 1,
                ROUTING = 2,
                FOLLOWING = 3,
            };

            RouteStateWorker() = default;

            /**
             * \brief Process route event based on designed state machine diagram
             * \param event Incoming route event
             */
            void on_route_event(RouteEvent event);

            /**
             * \brief Get current route state machine state
             */
            RouteState get_route_state() const;

        private:

            // private local variable tracks the current route satte
            RouteState state_ = RouteState::LOADING;

    };

}