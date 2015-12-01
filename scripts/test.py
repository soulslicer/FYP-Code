import numpy as np
import cv2

import itertools
from scipy import linalg
import matplotlib.pyplot as plt
import matplotlib as mpl

from sklearn import mixture

import time

N = 1000
State1 = np.asarray([
            120 + 5.5 * np.random.randn(N), 
            120 + 5.5 * np.random.randn(N),
            np.ones(N),
            np.ones(N)
        ])
State2 = np.asarray([
            320 + 5.5 * np.random.randn(N), 
            320 + 5.5 * np.random.randn(N),
            np.ones(N),
            np.ones(N)
        ])
State3 = np.asarray([
            # np.random.uniform(0, 640, N), 
            # np.random.uniform(0, 480, N), 
            # np.ones(N),
            # np.ones(N)
            240 + 5.5 * np.random.randn(N), 
            240 + 5.5 * np.random.randn(N),
            np.ones(N),
            np.ones(N)
        ])
State = np.hstack([State1, State2, State3])

X = State[0:2,:].T
print X.shape

# np.random.seed(0)
# C = np.array([[0., -0.1], [1.7, .4]])
# X = np.r_[.7 * np.random.randn(N, 2) + np.array([-6, 3])]


# print X.T

# image=np.zeros((480, 640, 3))
# for i, (x0, x1) in enumerate(zip(State[0,:], State[1,:])):
#     cv2.circle(image,(int(x0),int(x1)), 2, (255,0,255), -1)

# while(1):
#     cv2.imshow("a", image)
#     cv2.waitKey(15)

##############################################################
currTime = int(round(time.time() * 1000))

lowest_bic = np.infty
bic = []
n_components_range = range(1, 3)
cv_types = ['spherical']
for cv_type in cv_types:
    for n_components in n_components_range:
        # Fit a mixture of Gaussians with EM
        gmm = mixture.GMM(n_components=n_components, covariance_type=cv_type)

        gmm.fit(X)
        bic.append(gmm.bic(X))
        if bic[-1] < lowest_bic:
            lowest_bic = bic[-1]
            best_gmm = gmm

best_gmm = mixture.DPGMM(n_components=4)
best_gmm.fit(X)

elapsed = int(round(time.time() * 1000)) - currTime    


print best_gmm
print "ELAPSED: " + str(elapsed)

bic = np.array(bic)
color_iter = itertools.cycle(['k', 'r', 'g', 'b', 'c', 'm', 'y'])
clf = best_gmm
bars = []

# Plot the BIC scores
spl = plt.subplot(2, 1, 1)
for i, (cv_type, color) in enumerate(zip(cv_types, color_iter)):
    xpos = np.array(n_components_range) + .2 * (i - 2)
    bars.append(plt.bar(xpos, bic[i * len(n_components_range):
                                  (i + 1) * len(n_components_range)],
                        width=.2, color=color))
plt.xticks(n_components_range)
plt.ylim([bic.min() * 1.01 - .01 * bic.max(), bic.max()])
plt.title('BIC score per model')
xpos = np.mod(bic.argmin(), len(n_components_range)) + .65 +\
    .2 * np.floor(bic.argmin() / len(n_components_range))
plt.text(xpos, bic.min() * 0.97 + .03 * bic.max(), '*', fontsize=14)
spl.set_xlabel('Number of components')
spl.legend([b[0] for b in bars], cv_types)

# Plot the winner
splot = plt.subplot(2, 1, 2)
Y_ = clf.predict(X)
print Y_.shape
for i, (mean, color) in enumerate(zip(clf.means_, 
                                             color_iter)):
    # print covar
    # v, w = linalg.eigh(covar)
    # if not np.any(Y_ == i):
    #     continue
    # plt.scatter(X[Y_ == i, 0], X[Y_ == i, 1], .8, color=color)

    # # Plot an ellipse to show the Gaussian component
    # angle = np.arctan2(w[0][1], w[0][0])
    # angle = 180 * angle / np.pi  # convert to degrees
    # v *= 4
    # ell = mpl.patches.Ellipse(mean, v[0], v[1], 180 + angle, color=color)
    # ell.set_clip_box(splot.bbox)
    # ell.set_alpha(.5)
    # splot.add_artist(ell)

    # print covar
    # v, w = linalg.eigh(covar)
    if not np.any(Y_ == i):
        continue
    plt.scatter(X[Y_ == i, 0], X[Y_ == i, 1], .8, color=color)

    # Plot an ellipse to show the Gaussian component
    # angle = np.arctan2(w[0][1], w[0][0])
    # angle = 180 * angle / np.pi  # convert to degrees
    # v *= 4
    ell = mpl.patches.Ellipse(mean, 1, 1, 0, color=color)
    ell.set_clip_box(splot.bbox)
    ell.set_alpha(.5)
    splot.add_artist(ell)

plt.xlim(0, 640)
plt.ylim(0, 480)
# plt.xlim(-10, 10)
# plt.ylim(-3, 6)
plt.xticks(())
plt.yticks(())
plt.title('Selected GMM: full model, 2 components')
plt.subplots_adjust(hspace=.35, bottom=.02)
plt.show()
##############################################################


# N = 5
# State = np.asarray([
#             np.random.uniform(-4.7, 4.7, N), 
#             np.random.uniform(-4.7, 4.7, N),
#             np.ones(N),
#             np.ones(N)
#         ])
# print State


# A=np.random.randint(5,size=(10,3))
# print A

# Hits = int(N*0.2)
# randomized = np.random.choice(range(N), Hits, replace=False)
# print randomized
# State[:,randomized] = np.asarray([
#     np.random.uniform(-4.7, 4.7, Hits), 
#     np.random.uniform(-4.7, 4.7, Hits),
#     np.zeros(Hits),
#     np.zeros(Hits)
# ])

# print State

# State = np.delete(State, randomized, axis=1)

# print State

# WORKING
# randomized = np.random.choice(range(N), 3, replace=False)
# print randomized

# State[:,randomized] = np.asarray([
#             np.random.uniform(-4.7, 4.7, 3), 
#             np.random.uniform(-4.7, 4.7, 3),
#             np.zeros(3),
#             np.zeros(3)
#         ])
# print State 
##################

# State[:,randomized] = np.random.uniform(-4.7, 4.7, (4,3))
# print State 

# print np.random.uniform(-4.7, 4.7, (4,3))

# A[np.random.choice(A.shape[0], 10)] = 10
# print A



# from Algorithms import *
# # import json

# # (-1.6609013138627038, 10.005995432479482)
# # [134 171]
# # (-1.5609013138627037, 10.005995432479482)
# # [138 171]
# # (-1.4609013138627036, 10.005995432479482)
# # [0 0]

# # for i in range(354, 370):
# #     print "Trying " + str(i) + " " + str(1001)
# #     print sonar_xym_test(i,1001)

# # print sonar_pixel_map(0,0)
# # print sonar_pixel_map(398-1,0)
# # print sonar_pixel_map(398-1,480-1)
# # print sonar_pixel_map(0,480-1)

# # print sonar_xym_map(-1.6609013138627038,10.005995432479482)
# # print sonar_xym_map(-1.4609013138627036,10.005995432479482)

# # print sonar_xym_test(354, 1001)




# mapper = Mapper()

# print mapper.vehicle3DPoint(RSonar=4.032452, ThetaSonar=-13.6097, VCamera=234.9824, Pitch=2.75)

# # print mapper.vehicle3DPoint(RSonar=4.032452, ThetaSonar=-5.635941, VCamera=229.9838, Pitch=3.99)


# #-0.3821

# # for i in range(1,20):
# #     print mapper.pixelMap(RSonar=5, ThetaSonar=20, YSonar=1)