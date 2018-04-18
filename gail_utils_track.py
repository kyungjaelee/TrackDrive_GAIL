import numpy as np
import scipy.signal

def load_trajs_mat(matname = './track_demonstration.mat'):
    l = scipy.io.loadmat(matname)
    trajs_mat = l['trajs4TF']

    demonstrations = []
    for observes, actions in zip(trajs_mat['observes'],trajs_mat['actions']):
        demonstrations.append({
            'observes':observes[0],
            'actions':actions[0]
        })
    obs_dim = demonstrations[0]['observes'].shape[1]
    act_dim = demonstrations[0]['actions'].shape[1]
    print('{} trajs are converted into python type'.format(len(demonstrations)))
    print('observation dim : {}, action dim : {}\n'.format(obs_dim,act_dim))
    return demonstrations, obs_dim, act_dim

def subsamples(demonstrations, n_demo):
    sub_demonstrations = []
    n_total = 0
    for d in demonstrations:
        n_total+=len(d['actions'])
        sub_demonstrations.append(d)
        if n_total > n_demo:
            break
    return sub_demonstrations

def discount(x, gamma):
    """ Calculate discounted forward sum of a sequence at each point """
    return scipy.signal.lfilter([1.0], [1.0, -gamma], x[::-1])[::-1]

def add_disc_sum_rew(trajectories, gamma):
    """ Adds discounted sum of rewards to all time steps of all trajectories
    Args:
        trajectories: as returned by run_policy()
        gamma: discount
    Returns:
        None (mutates trajectories dictionary to add 'disc_sum_rew')
    """
    for trajectory in trajectories:
        if gamma < 0.999:  # don't scale for gamma ~= 1
            rewards = trajectory['rewards'] * (1 - gamma)
        else:
            rewards = trajectory['rewards']
        disc_sum_rew = discount(rewards, gamma)
        trajectory['disc_sum_rew'] = disc_sum_rew

def add_rew(trajectories, rew_func):
    for trajectory in trajectories:
        observes = trajectory['observes']
        actions = trajectory['actions']
        observes_actions = np.concatenate([observes,actions],axis=1)
        rewards = rew_func.predict(observes_actions)
        rewards = np.reshape(rewards,[-1,])
        trajectory['rewards'] = rewards
    return trajectories

def add_value(trajectories, val_func):
    """ Adds estimated value to all time steps of all trajectories
    Args:
        trajectories: as returned by run_policy()
        val_func: object with predict() method, takes observations
            and returns predicted state value
    Returns:
        None (mutates trajectories dictionary to add 'values')
    """
    for trajectory in trajectories:
        observes = trajectory['observes']
        values = val_func.predict(observes)
        values = np.reshape(values,[-1,])
        trajectory['values'] = values

def add_gae(trajectories, gamma, lam):
    for trajectory in trajectories:
        if gamma < 0.999:  # don't scale for gamma ~= 1
            rewards = trajectory['rewards'] * (1 - gamma)
        else:
            rewards = trajectory['rewards']
        values = trajectory['values']
        # temporal differences
        tds = rewards - values + np.append(values[1:] * gamma, 0)
        advantages = discount(tds, gamma * lam)
        trajectory['advantages'] = advantages

def build_train_set(trajectories):
    """
    Args:
        trajectories: trajectories after processing by add_disc_sum_rew(),
            add_value(), and add_gae()
    Returns: 4-tuple of NumPy arrays
        observes: shape = (N, obs_dim)
        actions: shape = (N, act_dim)
        advantages: shape = (N,)
        disc_sum_rew: shape = (N,)
    """
    observes = np.concatenate([t['observes'] for t in trajectories])
    actions = np.concatenate([t['actions'] for t in trajectories])
    disc_sum_rew = np.concatenate([t['disc_sum_rew'] for t in trajectories])
    advantages = np.concatenate([t['advantages'] for t in trajectories])
    # normalize advantages
    advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-6)

    return observes, actions, advantages, disc_sum_rew

def build_train_set_for_rew(trajectories,demonstrations):
    
    real_observes = np.concatenate([d['observes'] for d in demonstrations])
    real_actions = np.concatenate([d['actions'] for d in demonstrations])
    obs_act_exp = np.concatenate([real_observes,real_actions],axis=1)
    
    fake_observes = np.concatenate([t['observes'] for t in trajectories])
    fake_actions = np.concatenate([t['actions'] for t in trajectories])
    obs_act_gen = np.concatenate([fake_observes,fake_actions],axis=1)
    return obs_act_exp, obs_act_gen