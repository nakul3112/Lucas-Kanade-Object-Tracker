import numpy as np
import sys
import glob
import copy

try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
import cv2


def affine_warp(param):
    warp = np.hstack([np.eye(2), np.zeros((2, 1))]) + param.reshape((2, 3), order='F')
    return warp


def gamma_correction(image, gamma=1.0):
	inv_Gamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** inv_Gamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
	# apply gamma correction using the lookup table
	return cv2.LUT(image, table)


def lucas_kanade_algo(image, template, bbox, param):
    W = affine_warp(param)
    count = 0
    norm_param = 1
    threshold = 0.006
    
    cv2.imshow("image", image)
    cv2.imshow("template", template)

    while norm_param > threshold:
        count += 1
        warped = cv2.warpAffine(image, W, (0, 0), flags=cv2.INTER_CUBIC + cv2.WARP_INVERSE_MAP)
        warped = warped[bbox[0, 1]:bbox[2, 1], bbox[0, 0]:bbox[2, 0]]
        cv2.imshow("warp", warped)

        if np.linalg.norm(warped) < np.linalg.norm(template):
            print("gamma inc")
            image = gamma_correction(image, gamma=1.5)

        elif np.linalg.norm(warped) > np.linalg.norm(template):
            print("gamma reduce")
            image = gamma_correction(image, gamma=0.8)

        # Warping the current image
        warped = cv2.warpAffine(image, W, (0, 0), flags=cv2.INTER_CUBIC + cv2.WARP_INVERSE_MAP)
        warped = warped[bbox[0, 1]:bbox[2, 1], bbox[0, 0]:bbox[2, 0]]

        # Calculating the Image gradient in X direction
        grad_x = cv2.Sobel(np.float32(image), cv2.CV_64F, 1, 0, ksize = 5)
        # Warping the gradient_X image
        grad_x = cv2.warpAffine(grad_x, W, (0, 0), flags=cv2.INTER_CUBIC + cv2.WARP_INVERSE_MAP)
        grad_x = grad_x[bbox[0, 1]:bbox[2, 1], bbox[0, 0]:bbox[2, 0]]

        # Calculating the Image gradient in Y direction
        grad_y = cv2.Sobel(np.float32(image), cv2.CV_64F, 0, 1, ksize = 5)
        # Warping the gradient_Y image
        grad_y = cv2.warpAffine(grad_y, W, (0, 0), flags=cv2.INTER_CUBIC + cv2.WARP_INVERSE_MAP)
        grad_y = grad_y[bbox[0, 1]:bbox[2, 1], bbox[0, 0]:bbox[2, 0]]

        # Calculating the error between template and the warped image
        error = template.flatten().astype(np.int) - warped.flatten().astype(np.int)

        grid_x = np.asarray(list(range(grad_x.shape[1])))
        grid_y = np.asarray(list(range(grad_x.shape[0])))
        grid_x, grid_y = np.meshgrid(grid_x, grid_y)

        # Calculating the steepest descent images
        steep_desc_imgs = np.array([
             np.multiply(grad_x.flatten(), grid_x.flatten()), np.multiply(grad_y.flatten(), grid_x.flatten()),
             np.multiply(grad_x.flatten(), grid_y.flatten()), np.multiply(grad_y.flatten(), grid_y.flatten()),
             grad_x.flatten(), grad_y.flatten()
        ]).T

        # Calculating te Hessian(second-order derivative) matrix from steepest descent images
        hessian = np.dot(steep_desc_imgs.T, steep_desc_imgs)

        # Finally, updating the change in parameters(dp)
        dp = np.dot(np.linalg.pinv(hessian), np.dot(steep_desc_imgs.T, error))

        # Finding the norm of parameter vector
        norm_param = np.linalg.norm(dp)
        param = param+(dp*10)
        #req_w = affine_warp(param)
        if(count > 1000):
            break

    print("count : ", count)
    return param


def pyramid(img, levels):
    for l_down in range(levels):
        img = cv2.pyrDown(img)
    for l_up in range(levels):
        img = cv2.pyrUp(img)
    return img


images = []
imagesColor = []
for file in glob.glob("human/*.jpg"):
    inputImage = cv2.imread(file)
    grey = cv2.cvtColor(inputImage, cv2.COLOR_BGR2GRAY)
    images.append(grey)
    imagesColor.append(inputImage)

# Vase
#rect = np.array([[110, 80], [185, 80], [185, 160] , [110, 160]])
#rectDraw = np.array([[125, 90], [172, 90], [172, 150], [125, 150]])
# Car
# rect = np.array([[118, 100], [340, 100], [340, 280], [118, 280]])
# rectDraw = np.array([[118,100],[340,100],[340,280],[118,280]])
# Human
rect = np.array([[257, 292], [285, 292], [285, 361], [257, 361]])
rectDraw = np.array([[257, 292], [285, 292], [285, 361], [257, 361]])

p = np.zeros(6) 
template = images[0][rect[0,1]:rect[2,1], rect[0, 0]:rect[2, 0]]
boundingBox_new = copy.deepcopy(rect)

output_video = cv2.VideoWriter('car.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10,
       (images[0].shape[1], images[0].shape[0]))

for i in range(1, len(images)):
    print("Current Frame : ", i)
    
    for l in range(2, -1, -1):
        print("Level : ", l)
        pyr_temp_frame = pyramid(images[0], l)
        pyr_temp = pyr_temp_frame[rect[0, 1]:rect[2, 1], rect[0, 0]:rect[2, 0]]
        pyr_image = pyramid(images[i], l)
        p = lucas_kanade_algo(pyr_image, pyr_temp, rect, p)
    
    p = lucas_kanade_algo(images[i], template, rect, p)
    
    # Plot new bounding box
    w = affine_warp(p)
    rectDraw_ = np.dot(w, np.vstack((rectDraw.T, np.ones((1, 4))))).T
    rectTemp = rectDraw_.astype(np.int32)
    [x_max, y_max] = list(np.max(rectTemp, axis = 0).astype(np.int))
    [x_min, y_min] = list(np.min(rectTemp, axis = 0).astype(np.int))
    boundingBox_new = np.array([[x_min, y_min],
                     [x_max, y_min],
                     [x_max, y_max],
                     [x_min, y_max]])
    
    output_box = cv2.polylines(imagesColor[i], [rectTemp], True, (0, 255, 0), thickness = 5)

    cv2.imshow("Bounding Box", output_box)
    rectTemp = rect.astype(np.int32)
    rectTemp = rectTemp.reshape((-1, 1, 2))
    # cv2.imshow("Poly",cv2.polylines(imagesColor[i],[rectTemp],True,(255,0,0),thickness = 5))
    output_video.write(output_box)
    if cv2.waitKey(0) & 0xFF == ord('q'):
        break 
    
output_video.release()
cv2.waitKey(0)
cv2.destroyAllWindows()


