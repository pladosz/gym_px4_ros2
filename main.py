from envs.px4_env import SinglePx4UavEnv
import numpy as np
import time

if __name__ == "__main__":
    env = SinglePx4UavEnv()
    #send new command every 1 second
    print('sending random velocity commads')
    while True:
        action = np.random.uniform(low = -1,high = 1, size = (3,))
        env.step(action)
        time.sleep(1)
    #env.step(np.array([0.5, 0.5, 0.5]))