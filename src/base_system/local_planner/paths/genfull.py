import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splrep, splev
from scipy.spatial import KDTree
from mpl_toolkits.mplot3d import Axes3D

# Section 1: Generate Paths
def generate_paths():
    dis = 1.0
    angle = 27
    deltaAngle = angle / 3
    scale = 0.65

    pathStartAll = np.zeros((4, 0))
    pathAll = np.zeros((5, 0))
    pathList = np.zeros((5, 0))
    pathID = 0
    groupID = 0

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.box(True)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')

    print('\nGenerating paths\n')

    shift1_values = np.arange(-angle, angle + deltaAngle, deltaAngle)
    for shift1 in shift1_values:
        wayptsStart = np.array([[0, 0, 0],
                                [dis, shift1, 0]])

        pathStartR = np.arange(0, dis + 0.01, 0.01)
        tck = splrep(wayptsStart[:, 0], wayptsStart[:, 1], k=min(3, len(wayptsStart[:, 0]) - 1))
        pathStartShift = splev(pathStartR, tck)

        pathStartX = pathStartR * np.cos(np.deg2rad(pathStartShift))
        pathStartY = pathStartR * np.sin(np.deg2rad(pathStartShift))
        pathStartZ = np.zeros_like(pathStartX)

        pathStart = np.vstack((pathStartX, pathStartY, pathStartZ, np.ones_like(pathStartX) * groupID))
        pathStartAll = np.hstack((pathStartAll, pathStart))

        shift2_start = -angle * scale + shift1
        shift2_end = angle * scale + shift1
        deltaAngle2 = deltaAngle * scale
        shift2_values = np.arange(shift2_start, shift2_end + deltaAngle2, deltaAngle2)

        for shift2 in shift2_values:
            shift3_start = -angle * scale**2 + shift2
            shift3_end = angle * scale**2 + shift2
            deltaAngle3 = deltaAngle * scale**2
            shift3_values = np.arange(shift3_start, shift3_end + deltaAngle3, deltaAngle3)

            for shift3 in shift3_values:
                pathStartR_col = pathStartR[:, np.newaxis]
                pathStartShift_col = pathStartShift[:, np.newaxis]
                pathStartZ_col = pathStartZ[:, np.newaxis]

                pathStart_section = np.hstack((pathStartR_col, pathStartShift_col, pathStartZ_col))
                additional_waypts = np.array([[2 * dis, shift2, 0],
                                              [3 * dis - 0.001, shift3, 0],
                                              [3 * dis, shift3, 0]])

                waypts = np.vstack((pathStart_section, additional_waypts))

                pathR = np.arange(0, waypts[-1, 0] + 0.01, 0.01)
                tck = splrep(waypts[:, 0], waypts[:, 1], k=min(3, len(waypts[:, 0]) - 1))
                pathShift = splev(pathR, tck)

                pathX = pathR * np.cos(np.deg2rad(pathShift))
                pathY = pathR * np.sin(np.deg2rad(pathShift))
                pathZ = np.zeros_like(pathX)

                path = np.vstack((pathX, pathY, pathZ, np.ones_like(pathX) * pathID, np.ones_like(pathX) * groupID))
                pathAll = np.hstack((pathAll, path))
                pathList = np.hstack((pathList, np.array([[pathX[-1]], [pathY[-1]], [pathZ[-1]], [pathID], [groupID]])))

                pathID += 1

                ax.plot(pathX, pathY, pathZ)

        groupID += 1

    plt.show()

    # Save 'startPaths.ply'
    with open('startPaths.ply', 'w') as file:
        file.write('ply\n')
        file.write('format ascii 1.0\n')
        file.write(f'element vertex {pathStartAll.shape[1]}\n')
        file.write('property float x\n')
        file.write('property float y\n')
        file.write('property float z\n')
        file.write('property int group_id\n')
        file.write('end_header\n')
        for i in range(pathStartAll.shape[1]):
            file.write(f'{pathStartAll[0, i]} {pathStartAll[1, i]} {pathStartAll[2, i]} {int(pathStartAll[3, i])}\n')

    # Save 'paths.ply'
    with open('paths.ply', 'w') as file:
        file.write('ply\n')
        file.write('format ascii 1.0\n')
        file.write(f'element vertex {pathAll.shape[1]}\n')
        file.write('property float x\n')
        file.write('property float y\n')
        file.write('property float z\n')
        file.write('property int path_id\n')
        file.write('property int group_id\n')
        file.write('end_header\n')
        for i in range(pathAll.shape[1]):
            file.write(f'{pathAll[0, i]} {pathAll[1, i]} {pathAll[2, i]} {int(pathAll[3, i])} {int(pathAll[4, i])}\n')

    # Save 'pathList.ply'
    with open('pathList.ply', 'w') as file:
        file.write('ply\n')
        file.write('format ascii 1.0\n')
        file.write(f'element vertex {pathList.shape[1]}\n')
        file.write('property float end_x\n')
        file.write('property float end_y\n')
        file.write('property float end_z\n')
        file.write('property int path_id\n')
        file.write('property int group_id\n')
        file.write('end_header\n')
        for i in range(pathList.shape[1]):
            file.write(f'{pathList[0, i]} {pathList[1, i]} {pathList[2, i]} {int(pathList[3, i])} {int(pathList[4, i])}\n')

# Section 2: Find Correspondences
def find_correspondences():
    voxelSize = 0.02
    searchRadius = 0.45
    offsetX = 3.2
    offsetY = 4.5
    voxelNumX = 161
    voxelNumY = 451

    print('\nPreparing voxels\n')

    voxelPointNum = voxelNumX * voxelNumY
    voxelPoints = np.zeros((voxelPointNum, 2))

    indPoint = 0
    for indX in range(voxelNumX):
        x = offsetX - voxelSize * indX
        scaleY = x / offsetX + searchRadius / offsetY * (offsetX - x) / offsetX
        for indY in range(voxelNumY):
            y = scaleY * (offsetY - voxelSize * indY)
            voxelPoints[indPoint, 0] = x
            voxelPoints[indPoint, 1] = y
            indPoint += 1

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(voxelPoints[:, 0], voxelPoints[:, 1], np.zeros(voxelPointNum), 'k.')
    plt.show()

    print('\nCollision checking\n')

    pathAll = np.loadtxt('paths.ply', skiprows=9)
    pathAll = pathAll.T

    tree = KDTree(pathAll[:2, :].T)
    inds = tree.query_ball_point(voxelPoints, r=searchRadius)

    print('\nSaving correspondences\n')

    with open('correspondences.txt', 'w') as file:
        for i in range(voxelPointNum):
            file.write(f'{i} ')
            indVoxel = np.sort(inds[i])
            pathIndRec = -1
            for j in indVoxel:
                pathInd = pathAll[3, j]
                if pathInd == pathIndRec:
                    continue
                file.write(f'{int(pathInd)} ')
                pathIndRec = pathInd
            file.write('-1\n')
            if (i + 1) % 1000 == 0:
                print(f'Processed {i + 1} voxels')

    print('\nProcessing complete\n')

# Run the functions
generate_paths()
find_correspondences()

