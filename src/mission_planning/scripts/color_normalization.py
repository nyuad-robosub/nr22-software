import cv2
import matplotlib.pyplot as plt
import numpy as np
import scipy.ndimage as nd
import heapq

def box(img, r):
    """ O(1) box filter
        img - >= 2d image
        r   - radius of box filter
    """
    (rows, cols) = img.shape[:2]
    imDst = np.zeros_like(img)


    tile = [1] * img.ndim
    tile[0] = r
    imCum = np.cumsum(img, 0)
    imDst[0:r+1, :, ...] = imCum[r:2*r+1, :, ...]
    imDst[r+1:rows-r, :, ...] = imCum[2*r+1:rows, :, ...] - imCum[0:rows-2*r-1, :, ...]
    imDst[rows-r:rows, :, ...] = np.tile(imCum[rows-1:rows, :, ...], tile) - imCum[rows-2*r-1:rows-r-1, :, ...]

    tile = [1] * img.ndim
    tile[1] = r
    imCum = np.cumsum(imDst, 1)
    imDst[:, 0:r+1, ...] = imCum[:, r:2*r+1, ...]
    imDst[:, r+1:cols-r, ...] = imCum[:, 2*r+1 : cols, ...] - imCum[:, 0 : cols-2*r-1, ...]
    imDst[:, cols-r: cols, ...] = np.tile(imCum[:, cols-1:cols, ...], tile) - imCum[:, cols-2*r-1 : cols-r-1, ...]

    return imDst

#https://github.com/swehrwein/python-guided-filter/blob/master/gf.py
def _gf_gray(I, p, r, eps, s=None):
    """ grayscale (fast) guided filter
        I - guide image (1 channel)
        p - filter input (1 channel)
        r - window raidus
        eps - regularization (roughly, allowable variance of non-edge noise)
        s - subsampling factor for fast guided filter
    """
    if s is not None:
        Isub = nd.zoom(I, 1/s, order=1)
        Psub = nd.zoom(p, 1/s, order=1)
        r = round(r / s)
    else:
        Isub = I
        Psub = p


    (rows, cols) = Isub.shape

    N = box(np.ones([rows, cols]), r)

    meanI = box(Isub, r) / N
    meanP = box(Psub, r) / N
    corrI = box(Isub * Isub, r) / N
    corrIp = box(Isub * Psub, r) / N
    varI = corrI - meanI * meanI
    covIp = corrIp - meanI * meanP


    a = covIp / (varI + eps)
    b = meanP - a * meanI

    meanA = box(a, r) / N
    meanB = box(b, r) / N

    if s is not None:
        meanA = nd.zoom(meanA, s, order=1)
        meanB = nd.zoom(meanB, s, order=1)

    q = meanA * I + meanB
    return q

def dark_channel(i):
    M, N, _ = i.shape #Dimensions of the image array
    #Padding an array with the values of i
    padded = np.pad(i, ((int(w/2), int(w/2)), (int (w/2),int (w/2)), (0, 0)), 'edge')
    #Creating the dark channel array
    dark = np.zeros ((M, N))
    #Selecting the lowest intensity pixels out of the three channels of i
    for i, j in np.ndindex (dark.shape) :
        dark[i, j] = np.min (padded[i:i + w, j:j + w, :])
    return dark

def bright_channel(i):
    M, N, _ = i.shape #Dimensions of the image array
    #Padding an array with the values of i
    padded = np.pad(i, ((int(w/2), int (w/2)), (int (w/2), int (w/2)), (0, 0)),'edge')
    #Creating the bright channel array
    bright = np.zeros((M, N))
    #Selecting the highest intensity pixels out of the three channels
    for i, j in np.ndindex (bright.shape):
        bright[i, j] = np.max(padded[i:i + w, j:j + w, :])
    return bright

def channel_intensities(image):

    #Reading the 3 channels of the image
    b, g, r = cv2.split (image)

    #Calculating the mean intensity of each channel

    t = image.size / 3
    bx = float (np.sum(b)) / t
    gx = float (np.sum(g)) / t
    rx = float (np.sum(r)) / t

    #Identifying the indexes of the maximum, medium and minimum channel intensities

    var = {bx: bi,gx:gi,rx:ri}
    cmax = var.get (max (var))
    cmin = var.get (min(var))

    if ((cmax==1 or cmax==2) and (cmin==1 or cmin==2)):
        cmid = 0
    if ((cmax==0 or cmax==2) and (cmin==0 or cmin==2)):
        cmid = 1
    if ((cmax==0 or cmax==1) and (cmin==0 or cmin==1)):
        cmid = 2
    
    return cmax, cmid, cmin, bx,gx,rx, b, g,r

def bgsubr (i, bright) :
    M, N = bright.shape #Dimensions of the image array
    #Getting the indexes of the maximum, medium and minmum color channels
    cmax, cmid, cmin, _, _, _, _, _, _ = channel_intensities(image)
    #Creating the maximum color difference arrav
    bgsubrr = np.zeros((M, N))
    #Seprating i into the three color channels accordingly
    arrcmax = i[..., cmax]

    arrcmid = i[..., cmid]
    arrcmin = i[..., cmid]
    #Calculating the maximum channel difference in each pixel
    for mi in range(M):
        for ni in range(N):
            bgsubrr[mi][ni] = 1 - max(max(arrcmax[mi][ni]-arrcmin[mi][ni], 0), max(arrcmid[mi][ni]-arrcmin[mi][ni],0))
            
    return bgsubrr

def atmospheric_light(i, ibright):
    M, N = ibright.shape #Dimensions of the rectified bright channel array
    at = np. empty(3) #Atmospheric light 
    selectvar = [] #Array used to get the variance of the darkest pixels

    #Storing the 3D input array into 2D array
    flati = i.reshape (M*N, 3)

    #Extracting the gray filter image
    gray= cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = np.float64 (gray)/255

    #Producing the variance within a block for each pixel from gray image
    win_var = nd.generic_filter(gray, np.var, size = 3)
    minvar=256

    #Finding the top 1%*M*N darkest pixels
    flatbright = ibright.ravel()
    top = heapq.nsmallest(int (M*N*0.1), flatbright)

    #Finding the dark pixel with the miminmum variance intensity
    a = np.where(np.isin(ibright, top))
    for n in range (0, len(a[0])):
        (b,c) = (a[0][n], a[1][n])
        selectvar.append (win_var [b,c])
        if (minvar>np.amin(selectvar)):
            minvar = np.amin(selectvar)
            ib, ic = b, c
            if (minvar == 0): 
                break
    #Getting the atmospheric light intensity

    at[0] = i[ib,ic,0]
    at[1] = i[ib,ic,1]
    at[2] = i[ib,ic,2]

    return at

def rectify_bright (bgsubr,bright):
    #Calculating the saturation channel of the image
    rgb = cv2. cvtColor(image, cv2.COLOR_BGR2RGB)
    hsv = cv2. cvtColor(rgb, cv2.COLOR_RGB2HSV)

    #Calculating the coefficient lambda
    lambd = (hsv[...,1].max ())/255

    #Calculating the rectified bright channel
    ibright = (bright*lambd) + (bgsubr*(1-lambd))
    return ibright 
    
def initial_transmission(a, ibright):
    M, N = ibright.shape #Dimensions of the image array
    init = np.zeros ((M,N)) #Initial transmittance
    #Calulating the transmittance over each channel
    for i in range(3) :
        init = init + ((ibright-a[i])/(1.-a[i]))
        init = (init - np.min(init))/(np.max(init) - np.min(init))
    init = init/3
    #Calculating the average value of the transmittance
    return (init - np.min(init))/(np.max(init) - np.min(init))
    
def refined_transmission(init):
    refined = np.full_like(init, 0)
    
    #Extracting the gray filter image
    gray = cv2.cvtColor(image, cv2. COLOR_BGR2GRAY)
    gray = np.float64 (gray)/255
    r = 8
    eps = 0.05
    #refined = cv2.ximgproc.guidedFilter(gray, init)
    refined=_gf_gray(gray,init,r,eps)
    #refined = guidedFilter(gray,init)

    return refined

def restoration_image(i, a, refined):
    M, N, _ = i. shape
    #Broadcasting the refined transmission into a 3D array
    corrected = np.broadcast_to(refined[:,:,None], (refined.shape[0], refined.shape[1], 3))
    
    #Restoring the original image
    j = ((i-a)/corrected) + a
    return j

def histogram_equalization(j):
    M, N, _ = j. shape #Dimensions of the restored image
    #Creating the means of the color channels
    bluemean=float ("%0.5f" % (2))
    greenmean=float ("%0.5f" % (2)) 
    redmean =float("%0.5f" % (2))
    #Handling the case of negative pixels
    for mi in range (M):
        for ni in range(N) :
            if (j[mi,ni,0] <= 0):
                j[mi,ni,0] = 0
            if (j[mi,ni,1] <= 0):
                j[mi,ni,1] = 0
            if (j[mi,ni,2] <= 0):
                j[mi,ni, 2] = 0

    #Getting the means and arrays of each channel with any numbers of channe
    _, _, _, b,g,r, barr, garr, rarr = channel_intensities(j*255)

    #Converting the intensity range to [0,1]
    barr=barr/255
    garr=garr/255
    rarr=rarr/255

    #Assigning the wanted means from each channel
    bidx=0.5
    gidx=0.49
    ridx=0.49

    #Equalizing the blue channel
    if (bidx>0):
        bint = float("%0.5f" % (bidx))
        while bluemean != bint:
            bluemean = float("%0.5f" % (float ((np.sum(barr))) / (M*N)))
            powb = np.log(bint)/np.log (bluemean)
            barr = (barr) ** (powb)

    #Equalizing the green channel
    if (gidx>0):
        gint = float ("%0.5f" % (gidx))
        while greenmean != gint:
            greenmean = float("%0.5f" % (float ((np.sum (garr))) / (M*N)))
            powg = np.log(gint)/np.log (greenmean)
            garr = (garr) ** (powg)

    #Equalizing the red channel
    if (ridx>0):
        rint = float("%0.5f" % (ridx))
        while redmean != rint:
            redmean = float ("%0.5f" % (float ( (np. sum (rarr))) / (M*N)) )
            powr = np.log(rint)/ np.log(redmean)
            rarr = (rarr) ** (powr)

    #Combining the three channels into the new restored image
    for mi in range (M):
        for ni in range(N) :
            j[mi,ni,0]=barr[mi,ni]
            j[mi,ni,1]=garr[mi,ni]
            j[mi,ni,2]=rarr[mi,ni]

    return j

global w,bi,gi,ri,image
w = 15 #Window size
bi,gi,ri=0,1,2 #Color channels indexes

def get_normalized_image(image):
    #Converting image into numpy array
    i = np.asarray(image)
    i = i[:, :, :3]/255

    height, width, _ = i.shape
    bright = bright_channel(i)
    bgsubr = bgsubr(i, bright)
    ibright = rectify_bright(bgsubr,bright)
    a = atmospheric_light(i, bright)
    init = initial_transmission(a, ibright)
    white = np.full_like(bright, 0)

    refined = refined_transmission(init) #fucker
    j = restoration_image(i, a, refined)
    result = histogram_equalization(j)

    return result
    # plt.imshow(result)
    # image = cv2.cvtColor(result.astype('float32'), cv2.COLOR_BGR2RGB)

    # pixels = np.array(image)

    # plt.imshow(pixels)

# #Name of the file
# image = cv2.imread('/home/rami/Downloads/marker_304r.jpg')

