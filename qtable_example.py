import kiddeeEnv
import gym
from gym import spaces
from geometry_msgs.msg import Twist
import math
import numpy as np
import random



class myEnv(kiddeeEnv.kidDeeEnv):
    def __init__(self) -> None:
        super().__init__()
        self.action_space=spaces.Discrete(5) #0-> forward, 1-> left ,2-> backward,3-> right,4-> stop
        self.discrete_to_continuous_intelligent_mapping={0:[0.2,0.0],1:[0.0,math.pi/9],2:[-0.2,0.0],3:[0.0,-(math.pi/9)],4:[0.0,0.0]} #[Linear speed,angular speed
        self.num_stop=0
        self.last_loc=None
        self.state=0
        self.goal=[]
    def step(self,action):
        self.update_action(action)
        state,reward,done=self.update_state(action)
        info={}
        return state,reward,done,info
    def update_action(self,action):
        msg=self.create_vel_msg(self.discrete_to_continuous_intelligent_mapping[action])
        self.command_velocity.publish(msg)
    def create_vel_msg(self,speed):
        msg=Twist()
        msg.linear.x=speed[0]
        msg.angular.z=speed[1]
        return msg
    def reset(self):
        super().reset()
        self.num_stop=0
        self.state=0
        goalx=random.randrange(-2, 3)
        goaly=random.randrange(-2, 3)
        self.goal=np.array([goalx,goaly])
        state,_,_=self.update_state()
        return state
    def update_state(self,action=None):
        scan,odom=super().take_observation()
        lidar_range=np.array(scan.ranges)
        #print(lidar_range)
        done=False
        reward=0
        self.current=[odom.pose.pose.position.x,odom.pose.pose.position.y]
        current=np.array(self.current)
        dispower=np.power(current-self.goal,2)
        polar_distance=np.sqrt(np.sum(dispower))
        if lidar_range[0]<0.15 or lidar_range[90]<0.15 or lidar_range[180]<0.15 or lidar_range[270]<0.15:
            print('Yeah')
            self.state|=1<<0
            reward-= 1000
            done=True
            return self.state,reward,done
        if lidar_range[0]<0.20:
            self.state|=1<<1
            reward+=1
        if lidar_range[90]<0.20:
            self.state|=1<<2
            reward+=1
        if lidar_range[180]<0.20:
            self.state|=1<<3
            reward+=1
        if lidar_range[270]<0.20:
            self.state|=1<<4
            reward+=1
        if action==4:
            self.state|=1<<5
            self.num_stop+=1
            reward-=1
            if self.num_stop>=5:
                reward-=5
        if action==0:
            reward+=0.1
        if self.last_loc==self.current:
            reward-=100
        #print(self.goal.shape)
        #print(current.shape)
        #print(dispower)
        if polar_distance<=0.01:
            done=True
            reward+=2000
        #reward+=polar_distance/3
        self.last_loc=[odom.pose.pose.position.x,odom.pose.pose.position.y]
        return self.state,reward,done

def main():
    env=myEnv()
    q_table=np.zeros([64,env.action_space.n])
    lr=0.1
    gamma=0.9
    ep=1000
    reward_list=[]
    for i in range(ep):
        print(f'No ep:{i}')
        sum_reward=0
        state = env.reset()
        print(f'Goal:{env.goal[0]},{env.goal[1]}')
        while True:
            #print(state)
            action = np.argmax(q_table[state-1,:])
            next_state,reward,done,_=env.step(action)
            q_table[state-1,action]=q_table[state-1,action]+lr*(reward+gamma*np.max(q_table[next_state-1,:]))
            sum_reward+=reward
            state=next_state
            if done:
                break
        reward_list.append(sum_reward)
        print(f'reward:{sum_reward}')

if __name__ == '__main__':
    main()





