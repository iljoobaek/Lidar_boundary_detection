#!/usr/bin/env python
# coding: utf-8

import pandas as pd
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from PIL import Image
import glob
from sklearn.cluster import DBSCAN
from sklearn import metrics
from scipy.spatial import ConvexHull, convex_hull_plot_2d


fig = plt.figure(figsize=(5, 3))

ax1 = fig.add_subplot(2, 2, 1)
ax1.set_title("6 Priors")
ax2 = fig.add_subplot(2, 2, 2)
ax2.set_title("2 Priors")
ax3 = fig.add_subplot(2, 1, 2)
ax3.set_title("Kitti Image(Front Cam)")

# fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5, 3))
ax1.set(xlim=(-3, 50), ylim=(-50, 50))
ax2.set(xlim=(-3, 50), ylim=(-50, 50))

drive_path = "/home/tarang/Lidar_Project_Fall_2019_Tarang/data/kitti_data/"
drive_id = "2011_09_26_drive_0005_sync"
# drive_id = "2011_09_26_drive_0051_sync"
image_path = "image_00/data/"


def visualize_dbscan(db):
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    print("Estimated number of clusters: %d" % n_clusters_)
    print("Estimated number of noise points: %d" % n_noise_)
    unique_labels = set(labels)
    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [0, 0, 0, 1]

        class_member_mask = labels == k

        xy = X[class_member_mask & core_samples_mask]
        plt.plot(
            xy[:, 0],
            xy[:, 1],
            "o",
            markerfacecolor=tuple(col),
            markeredgecolor="k",
            markersize=14,
        )

    #     xy = X[class_member_mask & ~core_samples_mask]
    #     plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
    #              markeredgecolor='k', markersize=6)

    plt.title("Estimated number of clusters: %d" % n_clusters_)
    plt.show()


def getBoundary(points):
    added_vertices = [points.shape[0] - 1, points.shape[0] - 2]
    if len(points) < 3:
        return None, np.array([]).reshape((0, 2))
    hull = ConvexHull(points)
    vs = list(hull.vertices)
    for v in added_vertices:
        if v in vs:
            vs.remove(v)
    boundary = points[vs]
    if len(boundary) == 0:
        boundary = np.array(boundary).reshape((0, 2))
    return hull, boundary


def detect_boundary_2priors(df):
    # Convert to numpy and do DBScan clustering
    X = df.to_numpy()[:, :2]
    db = DBSCAN(eps=0.5, min_samples=2).fit(X)

    # Visualize dbscan clusters
    # visualize_dbscan(db)

    # print("Original Points:", df.to_numpy()[:, :2].shape[0])

    # visualize points that were clustered
    # pl.scatter(X[db.core_sample_indices_][:,0],X[db.core_sample_indices_][:,1])

    filtered_X = X[db.core_sample_indices_]

    # using 2 priors for now (y>0 and y<0)
    # prior1 = df[df.y>0].to_numpy()[:,:2]
    # prior2 = df[df.y<=0].to_numpy()[:,:2]
    prior1 = filtered_X[filtered_X[:, 1] > 0, :]
    prior2 = filtered_X[filtered_X[:, 1] <= 0, :]

    # # add points to prior
    # 0,maxY and maxX,maxY to prior1
    # 0, minY and maxX,minY to prior2

    maxX = np.max(filtered_X[:, 0])
    maxY = np.max(filtered_X[:, 1])
    minY = np.min(filtered_X[:, 1])
    # print(prior1.shape, prior2.shape)
    # print(maxX,maxY,minY)
    prior1 = np.vstack([prior1, np.array([[0, maxY], [maxX, maxY]])])
    prior2 = np.vstack([prior2, np.array([[0, minY], [maxX, minY]])])
    # print(prior1.shape, prior2.shape)

    # plot priors
    # pl.scatter(prior1[:,0],prior1[:,1])
    # pl.scatter(prior2[:,0],prior2[:,1])

    # convex hull for priors
    hull1 = ConvexHull(prior1)
    hull2 = ConvexHull(prior2)

    added_vertices_prior1 = [prior1.shape[0] - 1, prior1.shape[0] - 2]
    added_vertices_prior2 = [prior2.shape[0] - 1, prior2.shape[0] - 2]

    v1 = list(hull1.vertices)
    for v in added_vertices_prior1:
        v1.remove(v)

    v2 = list(hull2.vertices)
    for v in added_vertices_prior2:
        if v in v2:
            v2.remove(v)

    boundary1 = prior1[v1]
    boundary2 = prior2[v2]

    priors = [prior1, prior2]
    boundaries = [boundary1, boundary2]
    return priors, boundaries


def detect_boundary_6priors(df, xlim1=5, xlim2=15):  # split priors by X values
    # Convert to numpy and do DBScan clustering
    X = df.to_numpy()[:, :2]
    db = DBSCAN(eps=0.5, min_samples=2).fit(X)

    # Visualize dbscan clusters
    # visualize_dbscan(db)

    #     print("Original Points:",df.to_numpy()[:,:2].shape[0])

    # visualize points that were clustered
    # pl.scatter(X[db.core_sample_indices_][:,0],X[db.core_sample_indices_][:,1])

    filtered_X = X[db.core_sample_indices_]

    # using 2 priors for now (y>0 and y<0)
    # prior1 = df[df.y>0].to_numpy()[:,:2]
    # prior2 = df[df.y<=0].to_numpy()[:,:2]
    prior1_y = filtered_X[filtered_X[:, 1] > 0, :]
    prior2_y = filtered_X[filtered_X[:, 1] <= 0, :]

    """
    Priors 

    0    X1    X2     W      
    -------------------
    |    |     |      |
    | 1  |  2  |  3   |
    |    |     |      |
    0]>--+-----+------+
    |    |     |      |
    | 4  |  5  |  6   |
    |    |     |      |
    -------------------

    """

    #     print("Length of upper half:",len(prior1_y))
    prior1 = prior1_y[prior1_y[:, 0] < xlim1, :]
    prior2 = prior1_y[
        np.logical_and(prior1_y[:, 0] >= xlim1, prior1_y[:, 0] < xlim2), :
    ]
    prior3 = prior1_y[prior1_y[:, 0] >= xlim2, :]
    #     print("Length of upper priors:",prior1.shape,prior2.shape,prior3.shape)
    #
    #     print("Length of lower half:",len(prior2_y))
    prior4 = prior2_y[prior2_y[:, 0] < xlim1, :]
    prior5 = prior2_y[
        np.logical_and(prior2_y[:, 0] >= xlim1, prior2_y[:, 0] < xlim2), :
    ]
    prior6 = prior2_y[prior2_y[:, 0] >= xlim2, :]
    #     print("Length of lower priors:",prior4.shape,prior5.shape,prior6.shape)

    # # add points to prior
    # 0,maxY and maxX,maxY to prior1
    # 0, minY and maxX,minY to prior2

    maxX = np.max(filtered_X[:, 0])
    maxY = np.max(filtered_X[:, 1])
    minY = np.min(filtered_X[:, 1])
    # print(prior1.shape, prior2.shape)
    # print(maxX,maxY,minY)
    prior1 = np.vstack([prior1, np.array([[0, maxY], [xlim1, maxY]])])
    prior2 = np.vstack([prior2, np.array([[xlim1, maxY], [xlim2, maxY]])])
    prior3 = np.vstack([prior3, np.array([[xlim2, maxY], [maxX, maxY]])])

    prior4 = np.vstack([prior4, np.array([[0, minY], [xlim1, minY]])])
    prior5 = np.vstack([prior5, np.array([[xlim1, minY], [xlim2, minY]])])
    prior6 = np.vstack([prior6, np.array([[xlim2, minY], [maxX, minY]])])
    # print(prior1.shape, prior2.shape)

    # plot priors
    # pl.scatter(prior1[:,0],prior1[:,1])
    # pl.scatter(prior2[:,0],prior2[:,1])

    priors = [prior1, prior2, prior3, prior4, prior5, prior6]
    hulls = []
    boundaries = []
    for prior in priors:
        hull, boundary = getBoundary(prior)
        hulls.append(hull)
        boundaries.append(boundary)

    return priors, boundaries


def is_dist_matchable(boundary1, boundary2, thres):
    if len(boundary1) > 0 and len(boundary2) > 0:
        dist = np.sum(np.sqrt((boundary1[-1] - boundary2[0]) ** 2))
        return dist < thres
    else:
        return True


def merge_set(boundaries):
    merged = []
    DIST_THRES = 5  # metres
    boundaries0 = np.array(boundaries[0])
    boundaries1 = np.array(boundaries[1])
    boundaries2 = np.array(boundaries[2])
    if is_dist_matchable(boundaries0, boundaries1, DIST_THRES):
        if is_dist_matchable(boundaries1, boundaries2, DIST_THRES):
            merged.append(np.vstack([boundaries0, boundaries1, boundaries2]))
        else:
            merged.append(np.vstack([boundaries0, boundaries1]))
            merged.append(boundaries2)
    else:
        if is_dist_matchable(boundaries1, boundaries2, DIST_THRES):
            merged.append(boundaries0)
            merged.append(np.vstack([boundaries1, boundaries2]))
        else:
            merged.append(boundaries0)
            merged.append(boundaries1)
            merged.append(boundaries2)
    return merged


def merge_boundaries(boundaries):
    """ 
    `boundaries` is an array of 6 boundaries(1 for each prior)
    This function matches the upper and lower halves based on a distance metric
    """
    upper_boundaries = merge_set(boundaries[:3])
    lower_boundaries = merge_set(boundaries[3:])
    return upper_boundaries + lower_boundaries


# visualize multiple frames
def detect_in_frame(filename, mode="6priors"):
    print(filename)
    df = pd.read_csv(filename)
    #     print(len(df))
    if mode == "6priors":
        priors, boundaries = detect_boundary_6priors(df)
        boundaries = merge_boundaries(boundaries)
    elif mode == "2priors":
        priors, boundaries = detect_boundary_2priors(df)

    #     for idx,prior in enumerate(priors):
    #         # visualize points and line
    #         ax.scatter(prior[:,0],prior[:,1])
    #         if(len(boundaries[idx])>0):
    #             ax.plot(boundaries[idx][:,0],boundaries[idx][:,1])
    #         else:
    #             print("no boundary for this prior")
    # boundaries = np.vstack(boundaries)
    priors = np.vstack(priors)
    # print(priors.shape, boundaries.shape)
    # return priors, boundaries
    return priors, boundaries


#     ax.show()

frame_offset = 0
folder = os.path.join(drive_path, drive_id, image_path)
num_frames = len(glob.glob(folder + "*.png"))
# frames = range(0, 153)
# for f in frames:
#     filename = f"./frame_pointclouds/frame_{f}.csv"
#     detect_in_frame(filename)

# ax.subplot(211)
scat6 = ax1.scatter([], [], s=5)

line6 = []

for i in range(6):
    line6.append(ax1.plot([], [], lw=1)[0])

scat2 = ax2.scatter([], [], s=5)

line2 = []

for i in range(2):
    line2.append(ax2.plot([], [], lw=1)[0])

# ax.subplot(212)
# line1 = ax.plot([], [], lw=1)[0]
# line2 = ax.plot([], [], lw=1)[0]
# line3 = ax.plot([], [], lw=1)[0]
# line4 = ax.plot([], [], lw=1)[0]
# line5 = ax.plot([], [], lw=1)[0]


def get_image(id):
    folder = os.path.join(drive_path, drive_id, image_path)
    im_path = os.path.join(folder, f"{id:010}.png")
    im = Image.open(im_path)
    return im
    # img =


img = ax3.imshow(get_image(0))


def init():
    scat6.set_offsets([])
    scat2.set_offsets([])

    return (scat6,)


def animate(i):
    filename = f"../{drive_id}_vscan/frame_{frame_offset+i}.csv"
    priors6, boundaries6 = detect_in_frame(filename, mode="6priors")
    scat6.set_offsets(priors6)
    priors2, boundaries2 = detect_in_frame(filename, mode="2priors")
    scat2.set_offsets(priors2)
    for k in range(6):
        if k < len(boundaries6):
            line6[k].set_xdata(boundaries6[k][:, 0])
            line6[k].set_ydata(boundaries6[k][:, 1])
        else:
            line6[k].set_xdata([])
            line6[k].set_ydata([])
    for m in range(2):
        # if m < len(boundaries2):
        line2[m].set_xdata(boundaries2[m][:, 0])
        line2[m].set_ydata(boundaries2[m][:, 1])
    # else:
    #     line2[m].set_xdata([])
    #     line2[m].set_ydata([])
    img.set_array(get_image(i))
    return (scat6,)


anim = FuncAnimation(
    fig,
    animate,
    init_func=init,
    frames=num_frames,
    interval=500,
    blit=False,
    repeat=False,
)
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
import time

time.sleep(5)
plt.draw()
plt.show()
