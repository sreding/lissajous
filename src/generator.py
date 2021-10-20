import numpy as np
from scipy.stats.qmc import LatinHypercube




class BaseGenerator:
    def __init__(self, min_ax_1, max_ax_1, min_ax_2, max_ax_2, min_ax_3, max_ax_3, ax_1='x', ax_2='y', ax_3='z',
                 seed=263481, sequence_len=1000, LHS_sequence_length = 1000, d=7):
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
        if d < 7:
            self.static = True
        else:
            self.static = False
        self.sampler = LatinHypercube(d, seed=self.seed)

        self.sequence_len = sequence_len

        self.LHS_Sequence = self.sampler.random(LHS_sequence_length)
        self.idx = 0

    def next_numbers(self):
        seq_len = len(self.LHS_Sequence)
        if seq_len <= self.idx:
            self.LHS_Sequence = self.sampler.random(seq_len)

        # Adjust scaling to highest range
        angle_range = [self.max_x - self.min_x, self.max_y - self.min_y, self.max_z - self.min_z]
        max_deg = max(angle_range)


        a = self.LHS_Sequence[self.idx % seq_len, 0]*angle_range[0]/max_deg
        b = self.LHS_Sequence[self.idx % seq_len, 1]*angle_range[1]/max_deg
        c = self.LHS_Sequence[self.idx % seq_len, 2]*angle_range[2]/max_deg

        if self.static:
            self.idx += 1
            return a, b, c

        delta = np.pi / self.LHS_Sequence[self.idx % seq_len, 3]
        phi = np.pi / self.LHS_Sequence[self.idx % seq_len, 4]

        n = min(self.LHS_Sequence[self.idx % seq_len, 5],self.LHS_Sequence[self.idx % seq_len, 6])*10+3
        m = max(self.LHS_Sequence[self.idx % seq_len, 5], self.LHS_Sequence[self.idx % seq_len, 6])*10+3
        self.idx += 1

        return a, b, c, delta, phi, int(n), int(m)

    def generate_next(self, sequence_len=None, save_as=''):
        raise NotImplementedError

    def normalize_to_vals(self,x, min_val=-20, max_val=20):
        x -= x.min()
        x /= x.max()
        x *= max_val - min_val
        x += min_val
        return x

class DynamicTrajectory(BaseGenerator):
    def __init__(self, min_ax_1, max_ax_1, min_ax_2, max_ax_2, min_ax_3, max_ax_3, ax_1='x', ax_2='y', ax_3='z',
                 seed=263481, sequence_len=1000, LHS_sequence_length=100):
        """
        Generate dynamic trajectories.
        :param min_ax_1: minimum value on the first axis
        :param max_ax_1: maximum value on the first axis
        :param min_ax_2: '' second axis
        :param max_ax_2: '' second axis
        :param min_ax_3: '' third axis
        :param max_ax_3: '' third axis
        :param ax_1: name attributed to first axis, e.g. pitch
        :param ax_2: name attributed to second axis, e.g. roll
        :param ax_3: name attributed to third axis, e.g. yaw
        :param seed: Seed to call numpy and scipy randomness generation
        :param sequence_len: # of point in the trajectory
        :param LHS_sequence_length: # of LHS samples generated at once. Reduce if space requirements are tight or only few curves are generated.
        """
        super(DynamicTrajectory, self).__init__(min_ax_1, max_ax_1, min_ax_2, max_ax_2, min_ax_3, max_ax_3, ax_1, ax_2, ax_3,
                                                  seed=seed,sequence_len=sequence_len, LHS_sequence_length=LHS_sequence_length)


    def generate_next(self, sequence_len=None, save_as=''):
        if sequence_len is None:
            sequence_len = self.sequence_len

        a, b, c, delta, phi, n, m = self.next_numbers()
        t = np.linspace(-np.pi, np.pi, sequence_len)

        x = self.normalize_to_vals(a*np.sin(n * t + delta), self.min_x, self.max_x)
        y = self.normalize_to_vals(b*np.sin(m * t), self.min_y, self.max_y)
        z = self.normalize_to_vals(c*np.sin(t + phi), self.min_z, self.max_z)

        xyz_as_table = np.concatenate((np.expand_dims(x, 1), np.expand_dims(y, 1), np.expand_dims(z, 1)),
                                      axis=1)
        if save_as != '':
            np.savetxt(save_as+'_' + str(self.idx) + '.csv', xyz_as_table, delimiter=',')

        return xyz_as_table


class StaticTrajectory(BaseGenerator):
    def __init__(self, min_ax_1, max_ax_1, min_ax_2, max_ax_2, min_ax_3, max_ax_3, ax_1='x', ax_2='y', ax_3='z',
                 seed=263481, sequence_len=1000, LHS_sequence_length=100):
        super(StaticTrajectory, self).__init__(min_ax_1, max_ax_1, min_ax_2, max_ax_2, min_ax_3, max_ax_3, ax_1, ax_2,
                                                ax_3,
                                                seed=seed, sequence_len=sequence_len,
                                                LHS_sequence_length=LHS_sequence_length, d=3)

    def generate_next(self, sequence_len=None, save_as=''):
        if sequence_len is None:
            sequence_len = self.sequence_len

        x, y, z = [],[],[]

        for _ in range(sequence_len):
            a, b, c = self.next_numbers()

            x.append(a)
            y.append(b)
            z.append(c)

        x = np.asarray(x)
        y = np.asarray(y)
        z = np.asarray(z)

        x = self.normalize_to_vals(x, self.min_x, self.max_x)
        y = self.normalize_to_vals(y, self.min_y, self.max_y)
        z = self.normalize_to_vals(z, self.min_z, self.max_z)

        xyz_as_table = np.concatenate((np.expand_dims(x, 1), np.expand_dims(y, 1), np.expand_dims(z, 1)),
                                      axis=1)
        if save_as != '':
            np.savetxt(save_as+'_' + str(self.idx) + '.csv', xyz_as_table, delimiter=',')

        return xyz_as_table












