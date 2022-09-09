#!/usr/bin/env python

def merge_list(x1, t1, x2, t2):
    Time = sorted(t1 + t2)
    result1 = []
    result2 = []
    result_time = []
    for t in Time:
        i1, i2 = 1, 1



        if t in t1:

            result1.append([x1[t1.index(t)], t])
            result_time.append(t)
        else:
            while t1[i1] < t:
                i1 = i1 + 1
                if i1 == len(t1) - 1 or i2 == len(t2) - 1:
                    break
            result1.append([intersection(t1[i1-1], t1[i1], t, x1[i1-1], x1[i1]), t])
            result_time.append(t)
        if t in t2:

            result2.append([x2[t2.index(t)], t])

        else:
            while t2[i2] < t:
                i2 = i2 + 1
                if i1 == len(t1) - 1 or i2 == len(t2) - 1:
                    break
            result2.append([intersection(t2[i2 - 1], t2[i2], t, x2[i2 - 1], x2[i2]), t])




    return result1, result2, result_time


def intersection(t1, t2, t_mid, v1, v2):
    print(t1, t2, t_mid, v1, v2)
    if t2 == t1:
        return v1
    return (t_mid - t1) * (v2 - v1) / (t2 - t1) + v1


if __name__ == "__main__":
    #print(intersection(1, 2, 1.5, 1, 2))
    r1, r2, r_time = merge_list(list(range(11, 51, 4)), list(range(1, 11)), [5, 15, 8, 22, 25, 30, 50],
                        [1.5, 3.5, 3.7, 5.5, 8, 8.5, 12])
    print('Result 1:', r1, 'len :', len(r1))
    print('Result 2:', r2, 'len :', len(r2))
    print('Time :', r_time, len(r_time))
