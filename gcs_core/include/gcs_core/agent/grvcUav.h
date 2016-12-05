//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 Carmelo J. Fernández-Agüera Tortosa
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
// Strategy testing and simulation environment for mbzirc competition
//----------------------------------------------------------------------------------------------------------------------
#ifndef _MBZIRC_AGENT_GRVCUAV_H_
#define _MBZIRC_AGENT_GRVCUAV_H_

#include "uav.h"
#include <grvc_quadrotor_uav/server.h>
#include <gcs_core/util/types.h>
#include <string>

namespace grvc {
	namespace mbzirc {

		class GrvcUav : public Uav {
		public:
			GrvcUav(const std::string& _uri, const WorldCoord& startPos);
			// Chases a target and takes it to the drop zone.
			void takeOff(double _height) override;
			void land() override;
			void goTo(const WorldCoord& _wp) override;
			void trackPath(const hal::WaypointList&) override;

			const WorldCoord& position() const override;
			/// A rectangle that is guaranteed to be scanned by the view frustrum of the camera.
			/// It should be the bigest rectangle inscribed in the intersection of the frustrum with the floor,
			/// or the closest (conservative) possible approximation
			Rectangle viewArea() const override;

			/// 666 TODO: Observations. Should include feedback on captured targets
		private:
			uav::Server::TakeOffService::Client*	take_off_srv;
			uav::Server::WaypointService::Client*	waypoint_srv;
			uav::Server::PathService::Client*		path_srv;
			uav::Server::LandService::Client*		land_srv;

			WorldCoord mPos;
		};
	}
}	// namespace grvc::mbzirc

#endif // _MBZIRC_AGENT_GRVCUAV_H_
