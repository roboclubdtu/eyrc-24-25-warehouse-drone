def pixel_to_whycon(img_i, img_j):
    sx = (12.66 + 0.06)/500
    sy = (12.57 + 0.09)/500

    goal_x= sx*img_i - 12.66
    goal_y= sy*img_j - 12.57
    
    return goal_x, goal_y
