import numpy as np

from scipy.stats.qmc import LatinHypercube
import math




class BaseGenerator:
    def __init__(self, min_ax_1, max_ax_1, min_ax_2, max_ax_2, min_ax_3, max_ax_3, ax_1='x', ax_2='y', ax_3='z',
                 seed=263481, sequence_len=1000, LHS_sequence_length = 1000):
        self.seed = seed
        self.min_x = min_ax_1
        self.max_x = max_ax_1
        self.min_y = min_ax_2
        self.max_y = max_ax_2
        self.min_z = min_ax_3
        self.max_z = max_ax_3

        self.x = ax_1
        self.y = ax_2
        self.z = ax_3

        np.random.seed(self.seed)
        self.sampler = LatinHypercube(d=5, seed=self.seed)

        self.sequence_len = sequence_len

        self.LHS_Sequence = self.sampler.random(LHS_sequence_length)
        self.idx = 0

    def next_numbers(self):
        seq_len = len(self.LHS_Sequence)
        if len(seq_len) <= self.idx:
            self.LHS_Sequence = self.sampler.random(len(seq_len))

        a = self.LHS_Sequence[self.idx % seq_len, 0]
        b = self.LHS_Sequence[self.idx % seq_len, 1]
        c = self.LHS_Sequence[self.idx % seq_len, 2]

        delta = np.pi / self.LHS_Sequence[self.idx % seq_len, 3]
        phi = np.pi / self.LHS_Sequence[self.idx % seq_len, 4]
        self.idx += 1

        return a, b, c, delta, phi

    def generate_next(self, sequence_len=None, save_as=''):
        raise NotImplementedError

    def normalize_to_vals(x, min_val=-20, max_val=20):
        x -= x.min()
        x /= x.max()
        x *= max_val - min_val
        x += min_val
        return x

class DynamicTrajectory(BaseGenerator):
    def __init__(self, min_ax_1, max_ax_1, min_ax_2, max_ax_2, min_ax_3, max_ax_3, ax_1='x', ax_2='y', ax_3='z',
                 seed=263481, sequence_len=1000, LHS_sequence_length=1000):
        super(DynamicTrajectory, self).__init__(min_ax_1, max_ax_1, min_ax_2, max_ax_2, min_ax_3, max_ax_3, ax_1='x', ax_2='y', ax_3='z',
                                                  seed=seed,sequence_len=sequence_len, LHS_sequence_length=LHS_sequence_length)

    def generate_next(self, sequence_len=None, save_as=''):
        if sequence_len is None:
            sequence_len = self.sequence_len

        a, b, c, delta, phi = self.next_numbers()
        t = np.linspace(-np.pi, np.pi, sequence_len)
        x = self.normalize_to_vals(np.sin(a * t + delta), self.min_x, self.max_x)
        y = self.normalize_to_vals(np.sin(b * t), self.min_y, self.max_y)
        z = self.normalize_to_vals(np.sin(c * t + phi), self.min_z, self.max_z)

        xyz_as_table = np.concatenate((np.expand_dims(x, 1), np.expand_dims(y, 1), np.expand_dims(z, 1)),
                                      axis=1)
        if save_as != '':
            np.savetxt(save_as+'_' + str(self.idx) + '.csv', xyz_as_table, delimiter=',')

        return xyz_as_table











