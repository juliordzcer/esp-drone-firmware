idf_component_register(SRCS "./deck/deck_digital.c" 
                            "./deck/deck_spi.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES crazyflie config deck)