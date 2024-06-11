import cv2
import numpy as np
import pygame
import os

varyable_name_list = ["timestep", "method", "gravity", "floor", "wall", 
"mass", "spring constant", "damping", "framerate", "resolution"] # FUCK DISPLAY SIZE!!!!!


spring_color = np.array([204, 61, 195])
red_node_color = np.array([210, 20, 20])
blue_node_color = np.array([20, 20, 210])
green_node_color = np.array([20, 210, 20])

def layer_imgs(BG, red_node_mask,blue_node_mask,green_node_mask, spring_mask):

    otp_img = BG * (1 - spring_mask) + spring_color * (spring_mask)
    otp_img = otp_img * (1 - red_node_mask) + red_node_color * (red_node_mask)
    otp_img = otp_img * (1 - green_node_mask) + green_node_color * (green_node_mask)
    otp_img = otp_img * (1 - blue_node_mask) + blue_node_color * (blue_node_mask)
    return otp_img

def refresh(BG, node_list, blue_idx, lock_list,connection_matrix):
    if len(blue_idx)!=0:
        blue_idx=blue_idx[0]
    else:
        blue_idx=-1
    red_node_mask=blue_node_mask= green_node_mask=spring_mask = np.zeros(BG.shape)
    for idx_main in range(len(node_list)):
        POS = node_list[idx_main]

        if idx_main==blue_idx:
            blue_node_mask = cv2.circle(blue_node_mask.copy(), POS, 3, (1.0, 1.0, 1.0), -1)
        if idx_main in lock_list:
            green_node_mask = cv2.circle(green_node_mask.copy(), POS, 3, (1.0, 1.0, 1.0), -1)

        else:
            red_node_mask = cv2.circle(red_node_mask.copy(), POS, 3, (1.0, 1.0, 1.0), -1)

        if connection_matrix.shape[0]!=0:
            for idx_partner in range(len(node_list)):
                if connection_matrix[idx_main, idx_partner]!=0:
                    spring_mask = cv2.line(spring_mask.copy(), node_list[idx_main], node_list[idx_partner], (1.0, 1.0, 1.0), 2)
    
    return layer_imgs(BG, red_node_mask,blue_node_mask,green_node_mask, spring_mask)




    

def user_create_softbody(params):
    print("-A new window has opened...")
    print("-right click to add a node")
    print("-left click to select a node")
    print("-to create a connection, select two nodes")
    print("-press z to remove the previously placed node")
    print("-press CTRL+S to save and quit to menu")


    pygame.init()

    X = Y = 512
    img_numpy = np.zeros([X, Y, 3], dtype=int)
    screen = pygame.display.set_mode((X, Y))

    ground_val = params[3]
    if ground_val!=None:
        GROUND_DISPLAY_COORDS=int(ground_val/32*X+X/2)
        img_numpy[:GROUND_DISPLAY_COORDS] = np.array([102, 102, 102])
    

    img_BG = np.rot90(img_numpy, -1)

    img_surf = pygame.surfarray.make_surface(img_BG)
    RUNNING = True
    screen.blit(img_surf, (0,0))

    node_list = []
    connections_array = np.array([])
    locklist=[]

    pygame.display.update()
    curr_selection_list = []
    line_append_list=[] # used only for display
    
    while RUNNING:

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                RUNNING=False

            if event.type == pygame.MOUSEBUTTONDOWN:
                CLICK_BOOL_R = pygame.mouse.get_pressed()[0]
                POS = pygame.mouse.get_pos()
                POS = np.array((POS[1], POS[0]))

                if CLICK_BOOL_R:




                    node_list.append(POS)
                    



                    AMOUNT_NEW = len(node_list)
                    if connections_array.shape[0]==0:
                        connections_array=np.array([[0.0]])
                    else:
                        connections_array_new = np.zeros([AMOUNT_NEW, AMOUNT_NEW])


                        connections_array_new[:(AMOUNT_NEW-1), :(AMOUNT_NEW-1)] = connections_array

                        connections_array = connections_array_new



                    img_numpy = refresh(img_BG, node_list, curr_selection_list, locklist, connections_array)

                    img_surf = pygame.surfarray.make_surface(img_numpy)
                    screen.blit(img_surf, (0,0))
                    pygame.display.update()
                
                CLICK_BOOL_L = pygame.mouse.get_pressed()[2]
                if CLICK_BOOL_L:

                        for idx, coord in enumerate(node_list):
                            dist = np.sum((coord-POS)**2)**0.5

                            if dist<8:

                                curr_selection_list.append(idx)

                                if len(curr_selection_list)==1:

                                    img_numpy = refresh(img_BG, node_list, curr_selection_list, locklist, connections_array)

                                    img_surf = pygame.surfarray.make_surface(img_numpy)
                                    screen.blit(img_surf, (0,0))
                                    pygame.display.update()

                                else:

                                    partner_idx = curr_selection_list[0]
                                    if partner_idx!=idx:
                                        distance_from_points = np.sum((node_list[idx]-node_list[partner_idx])**2)**0.5
                                        connections_array[partner_idx, idx] = distance_from_points
                                        connections_array[idx, partner_idx] = distance_from_points
                                        line_append_list.append((idx, partner_idx))

                                        curr_selection_list=[]

                                        img_numpy = refresh(img_BG, node_list, curr_selection_list, locklist, connections_array)
                                        img_surf = pygame.surfarray.make_surface(img_numpy)
                                        screen.blit(img_surf, (0,0))
                                        pygame.display.update()
                                        #line_append_list = np.array((node_list[idx],node_list[partner_idx]))


                             
            
            if event.type == pygame.KEYDOWN:
                if len(node_list)>0:
                        
                    if event.key == pygame.K_q:
                        curr_selection_list=[]
                        POS = pygame.mouse.get_pos()
                        POS = np.array((POS[1], POS[0]))
                        for idx, coord in enumerate(node_list):
                            dist = np.sum((coord-POS)**2)**0.5

                            if dist<8:
                                locklist.append(idx)


                                img_numpy = refresh(img_BG, node_list, curr_selection_list, locklist, connections_array)

                                img_surf = pygame.surfarray.make_surface(img_numpy)
                                screen.blit(img_surf, (0,0))
                                pygame.display.update()
                    
                    if event.key == pygame.K_w:
                        curr_selection_list=[]
                        if len(locklist)>0:
                            locklist.pop()

                            img_numpy = refresh(img_BG, node_list, curr_selection_list, locklist, connections_array)

                            img_surf = pygame.surfarray.make_surface(img_numpy)
                            screen.blit(img_surf, (0,0))
                            pygame.display.update()

                    if event.key == pygame.K_z:
                            curr_selection_list=[]


                            POS = node_list.pop()
                            AMOUNT_NEW = len(node_list)

                            connections_array = connections_array[:AMOUNT_NEW, :AMOUNT_NEW]

                            img_numpy = refresh(img_BG, node_list, curr_selection_list, locklist, connections_array)
                            img_surf = pygame.surfarray.make_surface(img_numpy)
                            screen.blit(img_surf, (0,0))
                            pygame.display.update()
                    
                    if event.key == pygame.K_a:
                        curr_selection_list=[]
                        if len(line_append_list)>0:
                            

                            IDXES = line_append_list.pop()
                            connections_array[IDXES[0], IDXES[1]] = 0
                            connections_array[IDXES[1], IDXES[0]] = 0
                            img_numpy = refresh(img_BG, node_list, curr_selection_list, locklist, connections_array)
                            img_surf = pygame.surfarray.make_surface(img_numpy)
                            screen.blit(img_surf, (0,0))
                            pygame.display.update()
            
            all_keys = pygame.key.get_pressed()
            if all_keys[1073742048] and all_keys[pygame.K_s]:
                print("----EXITING USER GUI----")
                RUNNING=False
                pygame.quit()
                node_list = np.array(node_list, dtype=float)*32/X-16
                #node_list = np.concatenate((node_list[:, 1].reshape(node_list.shape[0], 1), node_list[:,0].reshape(node_list.shape[0], 1)), axis=1)
                return node_list, 32*connections_array/X, np.array(locklist, dtype=int)

            
                    

            
            
if __name__ == "__main__":
    user_create_softbody(params = [0.001, "RK4", -9.81, -6.0, 16.0, 0.7, 400.0, -0.01, 12, 128])

