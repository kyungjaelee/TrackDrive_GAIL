import numpy as np
import tensorflow as tf
import scipy.io
from sklearn.utils import shuffle

def tf_reshape_mixture(tf_var,act_dim=None,n_mixture=None, name=None):
    return tf.transpose(tf.reshape(tf_var,[-1,n_mixture,act_dim]),perm=[0,2,1],name=name)

class Policy(object):
    def __init__(self, obs_dim, act_dim, kl_targ, ent_coeff=0.2, n_mixture=4, seed=0, mdn_weight='softmax', eps = 3.0, max_std = 1.0, lr=3e-4, clip_value = 0.2):
        """
        Args:
            obs_dim: num observation dimensions (int)
            act_dim: num action dimensions (int)
            kl_targ: target KL divergence between pi_old and pi_new
        """
        self.seed=seed
        self.mdn_weight = mdn_weight
        self.clip_value = 0.2
        self.beta = 1.0  # dynamically adjusted D_KL loss multiplier
        self.eta = 0.  # multiplier for D_KL-kl_targ hinge-squared loss
        self.kl_targ = kl_targ
        self.epochs = 10
        self.lr = lr 
        self.lr_multiplier = 1.0  # dynamically adjust lr when D_KL out of control
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.n_mixture = n_mixture
        self.max_std = max_std
        self.eps = eps
        self.ent_coeff = ent_coeff
        self._build_graph()
        self._init_session()

        """ Get variables """
        with self.g.as_default():
            _t_vars = tf.trainable_variables()
            self.t_vars = [var for var in _t_vars]
 
        """ Print """
        print ("Policy Network")
        print ("Obs dim:[%d],Act dim:[%d]"%(self.obs_dim,self.act_dim))
        print ("Trainable Variables (%02d)" % len(self.t_vars))
        for i in range(len(self.t_vars)):
            w_name  = self.t_vars[i].name
            w_shape = self.t_vars[i].get_shape().as_list()
            print (" [%02d] Name:[%s] Shape:[%s]" % (i,w_name,w_shape))
        print ("")
        
    def save_as_mat(self, file_name='mdn'):
        save_dict = {}
        for i in range(len(self.t_vars)):
            w_name = self.t_vars[i].name
            w_name = w_name.replace('/','_')
            w_name = w_name.replace(':0','')
            w_var = self.sess.run(self.t_vars[i])
            save_dict[w_name] = w_var
        save_dict['eps'] = self.eps
        save_dict['mdn_weight'] = self.mdn_weight
        save_dict['n_mixture'] = self.n_mixture
        save_dict['max_std'] = self.max_std
        save_dict['act_dim'] = self.act_dim
        save_dict['obs_dim'] = self.obs_dim
        
        path_name = './'+file_name+'_policy_weights.mat'
        scipy.io.savemat(path_name, mdict=save_dict)
        print('Weights are stored at {}'.format(path_name))
        
    def _build_graph(self):
        """ Build and initialize TensorFlow graph """
        self.g = tf.Graph()
        with self.g.as_default():
            self._placeholders()
            self._policy_nn()
            self._logprob()
            self._kl_entropy()
            self._loss_train_op()
            self.init = tf.global_variables_initializer()
            self.variables = tf.global_variables()
            
    def _placeholders(self):
        """ Input placeholders"""
        # observations, actions and advantages:
        self.obs_ph = tf.placeholder(tf.float32, (None, self.obs_dim), 'obs')
        self.act_ph = tf.placeholder(tf.float32, (None, self.act_dim), 'act')
        self.advantages_ph = tf.placeholder(tf.float32, (None,), 'advantages')
        self.eps_ph = tf.placeholder(tf.float32, (), 'epsilon')
        
        # strength of D_KL loss terms:
        self.beta_ph = tf.placeholder(tf.float32, (), 'beta')
        self.eta_ph = tf.placeholder(tf.float32, (), 'eta')
        
        # learning rate:
        self.lr_ph = tf.placeholder(tf.float32, (), 'eta')
        
        self.old_std_ph = tf.placeholder(tf.float32, (None, self.act_dim, self.n_mixture), 'old_std')
        self.old_means_ph = tf.placeholder(tf.float32, (None, self.act_dim, self.n_mixture), 'old_means')
        self.old_pi_ph = tf.placeholder(tf.float32, (None, self.n_mixture), 'old_pi')
    
    def _policy_nn(self):
        
        hid1_size = 128  # 10 empirically determined
        hid2_size = 128
        
        # 9e-4 empirically determined
        out = tf.layers.dense(self.obs_ph, hid1_size, tf.tanh,
                              kernel_initializer=tf.random_normal_initializer(stddev=0.01,seed=self.seed), name="h1")
        out = tf.layers.dense(out, hid2_size, tf.tanh,
                              kernel_initializer=tf.random_normal_initializer(stddev=0.01,seed=self.seed), name="h2")
        means = tf.layers.dense(out, self.act_dim*self.n_mixture,
                                kernel_initializer=tf.random_normal_initializer(stddev=0.01,seed=self.seed), 
#                                 bias_initializer=tf.random_uniform_initializer(minval=-3.0,maxval=3.0), 
                                name="flat_means")
#         self.means = tf.reshape(means,shape=[-1,self.act_dim,self.n_mixture], name="means")
        self.means = tf_reshape_mixture(means,act_dim=self.act_dim,n_mixture=self.n_mixture, name="means")
        
#         logits_std = tf.get_variable('logstd', [1, self.act_dim, self.n_mixture], tf.float32, tf.constant_initializer(-1.0))
        logits_std = tf.layers.dense(out, self.act_dim*self.n_mixture,
                                     kernel_initializer=tf.random_normal_initializer(stddev=0.01,seed=self.seed), 
                                     name="flat_logits_std")
#         self.std = tf.reshape(self.max_std*tf.sigmoid(logits_std),shape=[-1,self.act_dim,self.n_mixture], name="std")
        self.std = tf_reshape_mixture(self.max_std*tf.sigmoid(logits_std)+1e-8,act_dim=self.act_dim,n_mixture=self.n_mixture, name="std")
        self.std = self.std + self.eps_ph
        
        if self.mdn_weight is 'sparsemax':
            self.pi = tf.contrib.sparsemax.sparsemax(tf.layers.dense(out, self.n_mixture,
                                                    kernel_initializer=tf.random_normal_initializer(stddev=0.01,seed=self.seed), name="pi"))
        else:
            self.pi = tf.nn.softmax(tf.layers.dense(out, self.n_mixture,
                                                    kernel_initializer=tf.random_normal_initializer(stddev=0.01,seed=self.seed), name="pi"))
        
    def _logprob(self):
        """ Calculate log probabilities of a batch of observations & actions
        Calculates log probabilities using previous step's model parameters and
        new parameters being trained.
        """
        y = self.act_ph 
        mu = self.means
        sigma = self.std
        pi = self.pi
        
        quadratics = -0.5*tf.reduce_sum(tf.square((tf.tile(y[:,:,tf.newaxis],[1,1,self.n_mixture])-mu)/sigma),axis=1)
        logdet = -0.5*tf.reduce_sum(tf.log(sigma),axis=1)
        logconstant = - 0.5*self.act_dim*np.log(2.*np.pi)
        logpi = tf.log(pi + 1e-9)
        
        exponents = quadratics + logdet + logconstant + logpi
        logprobs = tf.reduce_logsumexp(exponents,axis=1)
        
        self.logp = logprobs

        old_sigma_ph = self.old_std_ph
        old_mu_ph = self.old_means_ph
        old_pi_ph = self.old_pi_ph
    
        old_quadratics = -0.5*tf.reduce_sum(tf.square((tf.tile(y[:,:,tf.newaxis],[1,1,self.n_mixture])-old_mu_ph)/old_sigma_ph),axis=1)
        old_logdet = -0.5*tf.reduce_sum(tf.log(old_sigma_ph),axis=1)
        old_logconstant = - 0.5*self.act_dim*np.log(2.*np.pi)
        old_logpi = tf.log(old_pi_ph + 1e-9)
        
        old_exponents = old_quadratics + old_logdet + old_logconstant + old_logpi
        old_logprobs = tf.reduce_logsumexp(old_exponents,axis=1)
        
        self.logp_old = old_logprobs
    
    def _kl_entropy(self):
        """
        Add to Graph:
            1. KL divergence between old and new distributions
            2. Entropy of present policy given states and actions
        https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Kullback.E2.80.93Leibler_divergence
        https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Entropy
        """
        std = self.std
        old_std = self.old_std_ph
        log_det_cov_old = tf.reduce_sum(tf.log(old_std),axis=1)
        log_det_cov_new = tf.reduce_sum(tf.log(std),axis=1)
        tr_old_new = tf.reduce_sum(old_std/std,axis=1)

        if self.mdn_weight is 'sparsemax':
            self.kl = tf.reduce_sum(0.5*(-self.old_pi_ph+self.pi)**2 + 0.5 * self.old_pi_ph*(log_det_cov_new - log_det_cov_old + tr_old_new + tf.reduce_sum(tf.square((self.means - self.old_means_ph)/std),axis=1) - self.act_dim),axis=1)
            self.kl = tf.reduce_mean(self.kl)

            self.entropy = tf.reduce_sum(self.pi*(0.5*(1-self.pi) + 0.5 * (self.act_dim * (np.log(2 * np.pi) + 1) +
                                  tf.reduce_sum(tf.log(std),axis=1))),axis=1)
            self.entropy = tf.reduce_mean(self.entropy)
        else:
            self.kl = tf.reduce_sum(self.old_pi_ph*(tf.log(self.old_pi_ph)-tf.log(self.pi)) + 0.5 * self.old_pi_ph*(log_det_cov_new - log_det_cov_old + tr_old_new + tf.reduce_sum(tf.square((self.means - self.old_means_ph)/std),axis=1) - self.act_dim),axis=1)
            self.kl = tf.reduce_mean(self.kl)

            self.entropy = tf.reduce_sum(self.pi*(-tf.log(self.pi) + 0.5*(self.act_dim * (np.log(2 * np.pi) + 1) +
                                  tf.reduce_sum(tf.log(std),axis=1))),axis=1)
            self.entropy = tf.reduce_mean(self.entropy)
    def _loss_train_op(self):
        """
        Three loss terms:
            1) standard policy gradient
            2) D_KL(pi_old || pi_new)
            3) Hinge loss on [D_KL - kl_targ]^2
        See: https://arxiv.org/pdf/1707.02286.pdf
        """
        ratios = tf.exp(self.logp - self.logp_old)
        clipped_ratios = tf.clip_by_value(ratios, clip_value_min=1 - self.clip_value, clip_value_max=1 + self.clip_value)
        loss_clip = tf.minimum(tf.multiply(self.advantages_ph, ratios), tf.multiply(self.advantages_ph, clipped_ratios))
        loss1 = -tf.reduce_mean(loss_clip)
        
        loss2 = tf.reduce_mean(self.beta_ph * self.kl)
        
        loss3 = self.eta_ph * tf.square(tf.maximum(0.0, self.kl - 2.0 * self.kl_targ))
        
        self.loss = loss1 + loss2 + loss3 - self.ent_coeff*self.entropy
        self.loss_p = -tf.reduce_mean(self.logp)
        
        optimizer = tf.train.AdamOptimizer(self.lr_ph)
        self.train_op = optimizer.minimize(self.loss)
        self.train_p_op = optimizer.minimize(self.loss_p)

    def _init_session(self):
        """Launch TensorFlow session and initialize variables"""
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        self.sess = tf.Session(config=config,graph=self.g)
        self.sess.run(self.init)

    def sample(self, obs):
        """Draw sample from policy distribution"""
        feed_dict = {self.obs_ph: obs, self.eps_ph:self.eps}
        pi, mu, sigma = self.sess.run([self.pi, self.means, self.std],feed_dict=feed_dict)
        
        n_points = np.shape(obs)[0]
        
        _y_sampled = np.zeros([n_points,self.act_dim])
        for i in range(n_points):
            k = np.random.choice(self.n_mixture,p=pi[i,:])
            _y_sampled[i,:] = mu[i,:,k] + np.random.randn(1,self.act_dim)*sigma[i,:,k]
        return _y_sampled
    
    def update(self, observes, actions, advantages, batch_size = 128):
        
        num_batches = max(observes.shape[0] // batch_size, 1)
        batch_size = observes.shape[0] // num_batches
        
        old_means_np, old_std_np, old_pi_np = self.sess.run([self.means, self.std, self.pi],{self.obs_ph: observes, self.eps_ph:self.eps})
        loss, kl, entropy = 0, 0, 0
        for e in range(self.epochs):
            # TODO: need to improve data pipeline - re-feeding data every epoch
            observes, actions, advantages, old_means_np, old_std_np, old_pi_np = shuffle(observes, actions, advantages, old_means_np, old_std_np, old_pi_np,random_state=0)
            for j in range(num_batches):
                start = j * batch_size
                end = (j + 1) * batch_size
                adv_batch = advantages[start:end].copy()
                adv_batch = (adv_batch - adv_batch.mean())#/(adv_batch.std()+1e-9)
                feed_dict = {self.obs_ph: observes[start:end,:],
                     self.act_ph: actions[start:end,:],
                     self.advantages_ph: adv_batch,
                     self.old_std_ph: old_std_np[start:end,:,:],
                     self.old_means_ph: old_means_np[start:end,:,:],
                     self.old_pi_ph: old_pi_np[start:end,:],
                     self.beta_ph: self.beta,
                     self.eta_ph: self.eta,
                     self.lr_ph: self.lr * self.lr_multiplier,
                     self.eps_ph: self.eps}        
                self.sess.run(self.train_op, feed_dict)
            
            feed_dict = {self.obs_ph: observes,
                 self.act_ph: actions,
                 self.advantages_ph: advantages,
                 self.old_std_ph: old_std_np,
                 self.old_means_ph: old_means_np,
                 self.old_pi_ph: old_pi_np,
                 self.beta_ph: self.beta,
                 self.eta_ph: self.eta,
                 self.lr_ph: self.lr * self.lr_multiplier,
                 self.eps_ph: self.eps}        
            loss, kl, entropy = self.sess.run([self.loss, self.kl, self.entropy], feed_dict)
#             if kl > self.kl_targ * 4:  # early stopping if D_KL diverges badly
#                 break
        # TODO: too many "magic numbers" in next 8 lines of code, need to clean up
        if kl > self.kl_targ * 2:  # servo beta to reach D_KL target
            self.beta = np.minimum(35, 1.5 * self.beta)  # max clip beta
            if self.beta > 30 and self.lr_multiplier > 0.1:
                self.lr_multiplier /= 1.5
        elif kl < self.kl_targ / 2:
            self.beta = np.maximum(1 / 35, self.beta / 1.5)  # min clip beta
            if self.beta < (1 / 30) and self.lr_multiplier < 10:
                self.lr_multiplier *= 1.5
        
        loss, kl, entropy = self.sess.run([self.loss, self.kl, self.entropy], feed_dict)
        return loss, kl, entropy
    
    def update_eps(self, decaying_rate = 0.99):
        self.eps *= decaying_rate
    
    def fit(self, x, y, batch_size=128):
        """ Fit model to current data batch + previous data batch
        Args:
            x: features
            y: target
            logger: logger to save training loss and % explained variance
        """
        num_batches = max(x.shape[0] // batch_size, 1)
        batch_size = x.shape[0] // num_batches
        
        x_train = x
        y_train = y
        for e in range(self.epochs):
            x_train, y_train = shuffle(x_train, y_train,random_state=0)
            for j in range(num_batches):
                start = j * batch_size
                end = (j + 1) * batch_size
                feed_dict = {self.obs_ph: x_train[start:end, :],
                             self.act_ph: y_train[start:end, :],
                             self.lr_ph: self.lr,
                             self.eps_ph: 0.0}
                _, l = self.sess.run([self.train_p_op, self.loss_p], feed_dict=feed_dict)
        feed_dict = {self.obs_ph: x_train, self.act_ph: y_train, self.lr_ph: self.lr, self.eps_ph: 0.0}
        loss_p = self.sess.run(self.loss_p, feed_dict=feed_dict)
        return loss_p
    
    def save_policy(self, path):
        saver = tf.train.Saver(self.variables)
        saver.save(self.sess, path+'/mdn'+str(self.n_mixture)+'_bc_policy.ckpt')

    def close_sess(self):
        """ Close TensorFlow session """
        self.sess.close()

class Value(object):
    """ NN-based state-value function """
    def __init__(self, obs_dim, lr=1e-4, seed=0):
        """
        Args:
            obs_dim: number of dimensions in observation vector (int)
        """
        self.seed = seed
        self.replay_buffer_x = None
        self.replay_buffer_y = None
        self.obs_dim = obs_dim
        self.epochs = 10
        self.lr = lr
        self._build_graph()
        
        """ Get variables """
        with self.g.as_default():
            _t_vars = tf.trainable_variables()
            self.t_vars = [var for var in _t_vars]
 
        """ Print """
        print ("Value Network")
        print ("Obs dim:[%d],Value dim:[%d]"%(self.obs_dim,1))
        print ("Trainable Variables (%02d)" % len(self.t_vars))
        for i in range(len(self.t_vars)):
            w_name  = self.t_vars[i].name
            w_shape = self.t_vars[i].get_shape().as_list()
            print (" [%02d] Name:[%s] Shape:[%s]" % (i,w_name,w_shape))
        print ("")
        
    def save_as_mat(self, file_name='mdn'):
        save_dict = {}
        for i in range(len(self.t_vars)):
            w_name = self.t_vars[i].name
            w_name = w_name.replace('/','_')
            w_name = w_name.replace(':0','')
            w_var = self.sess.run(self.t_vars[i])
            save_dict[w_name] = w_var
        path_name = './'+file_name+'_value_weights.mat'
        scipy.io.savemat(path_name, mdict=save_dict)
        print('Weights are stored at {}'.format(path_name))
        
    def _build_graph(self):
        """ Construct TensorFlow graph, including loss function, init op and train op """
        self.g = tf.Graph()
        with self.g.as_default():
            self.obs_ph = tf.placeholder(tf.float32, (None, self.obs_dim), 'obs_valfunc')
            self.val_ph = tf.placeholder(tf.float32, (None,), 'val_valfunc')
            # hid1 layer size is 10x obs_dim, hid3 size is 10, and hid2 is geometric mean
            hid1_size = 128
#             hid2_size = 128
            
            out = tf.layers.dense(self.obs_ph, hid1_size, tf.tanh,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=0.01,seed=self.seed), name="h1")
#             out = tf.layers.dense(out, hid2_size, tf.tanh,
#                                   kernel_initializer=tf.random_normal_initializer(
#                                       stddev=0.01,seed=self.seed), name="h2")
            out = tf.layers.dense(out, 1,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=0.01,seed=self.seed), name='output')
            self.out = tf.squeeze(out)
            self.loss = tf.reduce_mean(tf.square(self.out - self.val_ph))
            optimizer = tf.train.AdamOptimizer(self.lr)
            self.train_op = optimizer.minimize(self.loss)
            self.init = tf.global_variables_initializer()
        self.sess = tf.Session(graph=self.g)
        self.sess.run(self.init)

    def fit(self, x, y, batch_size=32):
        """ Fit model to current data batch + previous data batch
        Args:
            x: features
            y: target
            logger: logger to save training loss and % explained variance
        """
        num_batches = max(x.shape[0] // batch_size, 1)
        y_hat = self.predict(x)  # check explained variance prior to update
        
        if self.replay_buffer_x is None:
            x_train, y_train = x, y
        else:
            x_train = np.concatenate([x, self.replay_buffer_x])
            y_train = np.concatenate([y, self.replay_buffer_y])
        self.replay_buffer_x = x
        self.replay_buffer_y = y
        for e in range(self.epochs):
            x_train, y_train = shuffle(x_train, y_train, random_state=0)
            for j in range(num_batches):
                start = j * batch_size
                end = (j + 1) * batch_size
                feed_dict = {self.obs_ph: x_train[start:end, :],
                             self.val_ph: y_train[start:end]}
                _, l = self.sess.run([self.train_op, self.loss], feed_dict=feed_dict)
        y_hat = self.predict(x)
        loss = np.mean(np.square(y_hat - y))         # explained variance after update
        return loss

    def predict(self, x):
        """ Predict method """
        feed_dict = {self.obs_ph: x}
        y_hat = self.sess.run(self.out, feed_dict=feed_dict)

        return np.squeeze(y_hat)

    def close_sess(self):
        """ Close TensorFlow session """
        self.sess.close()
        

class Reward(object):
    """ NN-based state-value function """
    def __init__(self, obs_dim, act_dim, lr=1e-4, seed=0, epochs=100):
        """
        Args:
            obs_dim: number of dimensions in observation vector (int)
        """
        self.seed = seed
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.epochs = epochs
        self.lr = lr  # learning rate set in _build_graph()
        self._build_graph()

        """ Get variables """
        with self.g.as_default():
            _t_vars = tf.trainable_variables()
            self.t_vars = [var for var in _t_vars]
 
        """ Print """
        print ("Reward Network")
        print ("Obs dim:[%d],Act dim:[%d]"%(self.obs_dim,self.act_dim))
        print ("Trainable Variables (%02d)" % len(self.t_vars))
        for i in range(len(self.t_vars)):
            w_name  = self.t_vars[i].name
            w_shape = self.t_vars[i].get_shape().as_list()
            print (" [%02d] Name:[%s] Shape:[%s]" % (i,w_name,w_shape))
        print ("")
        
    def save_as_mat(self,file_name='mdn'):
        save_dict = {}
        for i in range(len(self.t_vars)):
            w_name = self.t_vars[i].name
            w_name = w_name.replace('/','_')
            w_name = w_name.replace(':0','')
            w_var = self.sess.run(self.t_vars[i])
            save_dict[w_name] = w_var
        path_name = './'+file_name+'_reward_weights.mat'
        scipy.io.savemat(path_name, mdict=save_dict)
        print('Weights are stored at {}'.format(path_name))
    
    def _build_graph(self):
        """ Construct TensorFlow graph, including loss function, init op and train op """
        self.g = tf.Graph()
        with self.g.as_default():
            self.obs_act_exp_ph = tf.placeholder(tf.float32, (None, self.obs_dim + self.act_dim), 'obs_act_exp_rewfunc')
            self.obs_act_gen_ph = tf.placeholder(tf.float32, (None, self.obs_dim + self.act_dim), 'obs_act_gen_rewfunc')
            
            hid1_size = 64  # 10 chosen empirically on 'Hopper-v1'
            hid2_size = 64  # 10 chosen empirically on 'Hopper-v1'
            
            # heuristic to set learning rate based on NN size (tuned on 'Hopper-v1')
            self.entcoeff = 1e-4
            
            # 3 hidden layers with tanh activations
            out = tf.layers.dense(self.obs_act_exp_ph, hid1_size, tf.tanh,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=0.01,seed=self.seed), name="h1")
            out = tf.layers.dense(out, hid2_size, tf.tanh,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=0.01,seed=self.seed), name="h2")
            exp_logits = tf.layers.dense(out, 1,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=0.01,seed=self.seed), name='output')
            self.exp_logits = tf.squeeze(exp_logits)
            
            out = tf.layers.dense(self.obs_act_gen_ph, hid1_size, tf.tanh,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=0.01,seed=self.seed), name="h1", reuse=True)
            out = tf.layers.dense(out, hid2_size, tf.tanh,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=0.01,seed=self.seed), name="h2", reuse=True)
            gen_logits = tf.layers.dense(out, 1,
                                  kernel_initializer=tf.random_normal_initializer(
                                      stddev=0.01,seed=self.seed), name='output', reuse=True)
            self.gen_logits = tf.squeeze(gen_logits)

            self.generator_acc = tf.reduce_mean(tf.to_float(tf.nn.sigmoid(gen_logits) < 0.5))
            self.expert_acc = tf.reduce_mean(tf.to_float(tf.nn.sigmoid(exp_logits) > 0.5))
            
            generator_loss = tf.nn.sigmoid_cross_entropy_with_logits(logits=gen_logits, labels=tf.zeros_like(gen_logits))
            generator_loss = tf.reduce_mean(generator_loss)
            
            expert_loss = tf.nn.sigmoid_cross_entropy_with_logits(logits=exp_logits, labels=tf.ones_like(exp_logits))
            expert_loss = tf.reduce_mean(expert_loss)
            
            logits = tf.concat([gen_logits, exp_logits], 0)
            entropy = tf.reduce_mean((1.-tf.nn.sigmoid(logits))*logits + tf.nn.softplus(-logits))
            entropy_loss = -self.entcoeff*entropy
    
#             Loss + Accuracy terms
            self.entropy = entropy
            self.expert_loss = expert_loss
            self.generator_loss = generator_loss
            self.loss = generator_loss + expert_loss + entropy_loss

#             Build Reward for policy
            self.reward = -tf.log(tf.nn.sigmoid(gen_logits)+1e-8)
            optimizer = tf.train.AdamOptimizer(self.lr)
            self.train_op = optimizer.minimize(self.loss)
            self.init = tf.global_variables_initializer()
        self.sess = tf.Session(graph=self.g)
        self.sess.run(self.init)

    def fit(self, obs_act_exp, obs_act_gen, batch_size=128):
        exp_data_size = obs_act_exp.shape[0]
        gen_data_size = obs_act_gen.shape[0]
        
        for e in range(self.epochs):
            exp_idx = np.random.permutation(exp_data_size)[:batch_size]
            gen_idx = np.random.permutation(gen_data_size)[:batch_size]
            feed_dict = {self.obs_act_gen_ph:obs_act_gen[gen_idx,:], self.obs_act_exp_ph:obs_act_exp[exp_idx,:]}
            self.sess.run(self.train_op, feed_dict=feed_dict)
            
        feed_dict = {self.obs_act_gen_ph:obs_act_gen, self.obs_act_exp_ph:obs_act_exp}
        loss, gen_acc, exp_acc,entropy,expert_loss,generator_loss = self.sess.run([self.loss,self.generator_acc,self.expert_acc,self.entropy,self.expert_loss,self.generator_loss], feed_dict=feed_dict)
        return loss, gen_acc, exp_acc,entropy,expert_loss,generator_loss
    
    def predict(self, x):
        """ Predict method """
        feed_dict = {self.obs_act_gen_ph: x}
        rew_hat = self.sess.run(self.reward, feed_dict=feed_dict)

        return np.squeeze(rew_hat)

    def close_sess(self):
        """ Close TensorFlow session """
        self.sess.close()
        