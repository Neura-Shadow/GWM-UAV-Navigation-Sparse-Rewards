// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// GWM P3 variant of the pinned v2.4.3 microxrce_agent.cpp (SHA-256 c3bdc86a...431e).
// Only main() differs: SIGINT/SIGTERM are blocked before the agent thread starts and
// consumed with sigwait(); the existing AgentInstance::stop() then ends the agent thread
// and main returns 0. The agent library and transports are the unchanged pinned build.

#include <uxr/agent/AgentInstance.hpp>

#include <csignal>
#include <pthread.h>

int main(int argc, char** argv)
{
    sigset_t stop_signals;
    sigemptyset(&stop_signals);
    sigaddset(&stop_signals, SIGINT);
    sigaddset(&stop_signals, SIGTERM);
    // Inherited by every agent thread, so only sigwait() below receives these signals.
    if (pthread_sigmask(SIG_BLOCK, &stop_signals, nullptr) != 0)
    {
        return 2;
    }

    eprosima::uxr::AgentInstance& agent_instance = agent_instance.getInstance();

    if (!agent_instance.create(argc, argv))
    {
        return 1;
    }

    int received = 0;
    if (sigwait(&stop_signals, &received) != 0)
    {
        return 3;
    }
    agent_instance.stop();
    return 0;
}
