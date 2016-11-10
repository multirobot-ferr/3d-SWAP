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
#include <gcs_core/agent/agent.h>
#include <gcs_core/agent/arenaUav.h>
#include <gcs_core/target/target.h>

namespace grvc {
	namespace mbzirc {
		//--------------------------------------------------------------------------------------------------------------
		Agent::Agent(const Vector2& startPos) {
			uav = new ArenaUav(hal::Vec3(startPos.x(), startPos.y(), flyHeight));
		}

		//--------------------------------------------------------------------------------------------------------------
		void Agent::captureStaticTarget(const Target::Desc& _target) {
			mCapturing = true;
			goTo(_target.pos);
		}

		//--------------------------------------------------------------------------------------------------------------
		void Agent::goTo(const Vector2& pos) {
			goalPos = pos;
			hal::Vector3 pos3 = Vector3(pos.x(), pos.y(), flyHeight);
			uav->goTo(pos3);
		}

		//--------------------------------------------------------------------------------------------------------------
		Vector2 Agent::position() const {
			auto pos3 = uav->position();
			return Vector2(pos3.x(), pos3.y());
		}

		//--------------------------------------------------------------------------------------------------------------
		void Agent::update() {
			if (mCapturing) {
				float distanceToGoal = (position() -  goalPos).norm();
				if(distanceToGoal <= closeEnough)
					mCapturing = false;
			}
		}
} }	// namespace grvc::mbzirc