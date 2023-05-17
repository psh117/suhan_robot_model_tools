from srmt.constraints.constraints import ParallelChainConstraint
import numpy as np

np.set_printoptions(precision=4, suppress=True, linewidth=300)


# pcc = ParallelChainConstraint()
for i in range(3,10):
    pcc2 = ParallelChainConstraint(links=3, chain_num=i)
    start = pcc2.get_start()
    goal = pcc2.get_goal()
    length = goal - start
    print('start: \n', start.reshape(-1,3).transpose())
    print('goal: \n', goal.reshape(-1,3).transpose())
    print('length: \n', length.reshape(-1,3).transpose())
    print('\n\n')
    print(pcc2.is_valid(pcc2.get_start()))
    print(pcc2.is_valid(pcc2.get_goal()))