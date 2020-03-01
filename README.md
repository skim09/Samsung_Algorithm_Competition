# Samsung_Algorithm_Competition
Submission for 2019 Algorithm Competition held at Samsung SDS (Korea and America)

Repository for algorithm competition submissions held during the summer of 2019 at Samsung SDS.

Interned at Samsung SDS America (SDSA) from June 2019 to August 2019, and during this summer I participated in the summer algorithm competition where participants submitted code to drive a simulated vehicle around a racecourse with various turns and obstacles. The competition was open to all employees in Samsung SDS, both in Korea and in America.
The competition was divided into two parts: reinforcement learning driving (folder rl_code_v7) and rule-based driving (folder rule_code_v8). The competition used Microsoft's AirSim application and custom racetracks.

### Reinforcement Learning
Samsung provided a framework code that implemented TenserFlow and Deep Q-Network (DQN) learning, and the participant was tasked with creating the reward function that would train the bot to pursue rewards and complete driving in the shortest amount of time and modifying the DQN parameters. The specific file that I edited was "rl_code_v7/rl/dqn_custom_client.py", where I designed my own algorithm that gave rewards based on its speed, distance from optimal position on the road, and angle at which it was travelling.
There were some speedbumps (pun intended) along the way that hindered my ability to come up with a complete learned algorithm. First, as an intern I was equipped with an outdated laptop that had limited hardware capabilities, and this proved to be troublesome as the simulator required the algorithm to consistently take in inputs and calculate outputs every tenth of a second, and my computer struggled to keep up with the speeds and could not run calculations at that speed. This applied to both the reinforcement learning and rule-based driving parts of the competition. Ultimately I asked for a better computer, and after being approved had a much newer model and had to reconfigure my code from the beginning.
Second, I was unable to successfully train a model to drive as I expected; it was completing the races but at a very slow speed. I realized that the problem was that I had not factored into the reward function time. It had been taking high speeds as part of its reward, which I expected to persuade the bot to drive faster, but in fact it was sacrificing a greater reward at each time point because it realized that it could have a much better overall reward by staying on the track as long as possible and accruing many small rewards, as opposed to big rewards. I could have solved this problem by inviting the car to care only about immediate rewards and not collect them, but also having rewards in the future and accruing rewards was an important part of the reward function. Another possibility would have been to factor in time as a sort of negative reward function, penalizing it for taking too much time. However, it was realized too late and my internship came to an end before I could successfully implement a proper solution.

### Rule-based
The rule-based competition also had a framework, but it was mostly just to connect the algorithm to the driving simulator (AirSim). My code can be found in "rule_code_v8/rule/driving_client.py". The algorithm controlled inputs to the car, namely accelerating, braking, and turning. Simplified, my calculated the forward angles of the track (available by simulator) and based on the angle of the forward pieces of track, slowed down and calculated the optimal location of the car to be in as it approached the curve. It also calculated optimal location to be on the track if there is an obstacle ahead. If there is an obstacle ahead and interfered with the calculated optimal location on the track it adjusted to avoid it. I implemented this by having the car always want to be in the center of the racetrack. However, if there are curves or obstacles, it "changes" the center to a different part of the road, and the car moved to the according center. There were a lot more specifics that required much fine-tuning, but that is the gist of the algorithm.

### Result
I ended up reaching the quarterfinals for rule-based driving competition, and got on the leaderboard (albeit not high up) for the reinforcement learning competition, and as reward for my efforts I was recognized by the CEO and received two Amazon gift cards for my work.