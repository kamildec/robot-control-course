#!/usr/bin/env python3

import cv2
import numpy as np

image = cv2.imread('view.png')

# image crop
image = image[0:400, :, :]
cv2.imshow('a', image)
cv2.waitKey()

# conversion to HSV colorspace
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# positive red hue margin
lower1 = np.array([0, 100, 50])
upper1 = np.array([10, 255, 255])
mask1 = cv2.inRange(hsv_image, lower1, upper1)

# negative red hue margin
lower2 = np.array([160,100,50])
upper2 = np.array([179,255,255])
mask2 = cv2.inRange(hsv_image, lower2, upper2)

mask = mask1
mask = mask1 + mask2

cv2.imshow('a', mask)
cv2.waitKey()

# trick for finding two centers
# wrong
# x=np.argmax(np.sum(mask, axis=0))
# y=np.argmax(np.sum(mask, axis=1))
mask_sum0=np.sum(mask, axis=0)
mask_sum1=np.sum(mask, axis=1)
x=round(np.mean(np.argwhere(mask_sum0 == np.amax(mask_sum0))))
y=round(np.mean(np.argwhere(mask_sum1 == np.amax(mask_sum1))))

print("center x", x)
print("center y", y)

mask[y][x] = 0
cv2.imshow('a', mask)
cv2.waitKey()


# blue bars
cv2.imshow('a', image)
cv2.waitKey()

blue = np.uint8([[[255, 0, 0]]])
hsv_blue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)
print(hsv_blue)
lower3 = np.array([110, 100, 100])
upper3 = np.array([130, 255, 255])
mask = cv2.inRange(hsv_image, lower3, upper3)

cv2.imshow('a', mask)
cv2.waitKey()
crop_pos = 350
left_bar = mask[:, :crop_pos]
right_bar = mask[:, crop_pos:]

left_bar_sum0=np.sum(left_bar, axis=0)
left_bar_sum1=np.sum(left_bar, axis=1)
left_bar_x=round(np.mean(np.argwhere(left_bar_sum0 == np.amax(left_bar_sum0))))
left_bar_y=round(np.mean(np.argwhere(left_bar_sum1 == np.amax(left_bar_sum1))))
print("center x", left_bar_x)
print("center y", left_bar_y)

right_bar_sum0=np.sum(right_bar, axis=0)
right_bar_sum1=np.sum(right_bar, axis=1)
right_bar_x=round(np.mean(np.argwhere(right_bar_sum0 == np.amax(right_bar_sum0)))) + crop_pos
right_bar_y=round(np.mean(np.argwhere(right_bar_sum1 == np.amax(right_bar_sum1))))
print("center x", right_bar_x)
print("center y", right_bar_y)

mask[left_bar_y][left_bar_x] = 0
mask[right_bar_y][right_bar_x] = 0

cv2.imshow('a', mask)
cv2.waitKey()

cv2.destroyAllWindows()
