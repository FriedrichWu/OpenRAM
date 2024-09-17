# See LICENSE for licensing information.
#
# Copyright (c) 2016-2024 Regents of the University of California, Santa Cruz
# All rights reserved.
#
from openram import debug
from openram.base.vector import vector
from openram.base.vector3d import vector3d
from openram import OPTS
from .graph import graph
from .graph_shape import graph_shape
from .router import router
import re

class io_pin_placer(router):
    """
    This is the signal escape router that uses the Hanan grid graph method.
    """

    def __init__(self, layers, design, bbox=None):

        # `router` is the base router class
        router.__init__(self, layers, design, bbox)

        # New pins are the side supply pins
        self.new_pins = {}
        
        # added_io_pins
        self.io_pins_added_left = []
        self.io_pins_added_right = []
        self.io_pins_added_up = []
        self.io_pins_added_down = []
             

    def get_closest_edge(self, point):
        """ Return a point's the closest edge and the edge's axis direction. """

        ll, ur = self.bbox

        # Snap the pin to the perimeter and break the iteration
        ll_diff_x = abs(point.x - ll.x)
        ll_diff_y = abs(point.y - ll.y)
        ur_diff_x = abs(point.x - ur.x)
        ur_diff_y = abs(point.y - ur.y)
        min_diff = min(ll_diff_x, ll_diff_y, ur_diff_x, ur_diff_y)

        if min_diff == ll_diff_x:
            return "left", True
        if min_diff == ll_diff_y:
            return "bottom", False
        if min_diff == ur_diff_x:
            return "right", True
        return "top", False
           

    def create_fake_pin(self, pin):
        """ Create a fake pin on the perimeter orthogonal to the given pin. """

        ll, ur = self.bbox
        c = pin.center()
        print("inside pin name")
        print("----------------------------------------------------------")
        print(pin.name)
        # Find the closest edge
        edge, vertical = self.get_closest_edge(c)
        offset = 0.95 + 0.19 # FIX: this is the magic number to overcome the ovetflow problem at the boundary, may need a method
        add_distance = 0
        # Keep the fake pin out of the SRAM layout are so that they won't be
        # blocked by previous signals if they're on the same orthogonal line
        if edge == "left":
            fake_center = vector(ll.x - self.track_wire * 2 + offset, c.y)
            is_too_close = any(abs(pin_added.center().y - fake_center.y) < (0.8 + self.half_wire * 4)for pin_added in self.io_pins_added_left)                  
            while is_too_close:         
                debug.warning("overlap, changing position") 
                add_distance = add_distance + 0.8 + self.half_wire * 4
                fake_center = vector(ll.x - self.track_wire * 2 + offset, c.y + add_distance)
                is_too_close = any(abs(pin_added.center().y - fake_center.y) < (0.8 + self.half_wire * 4)for pin_added in self.io_pins_added_left)  
                              
        if edge == "bottom":
            fake_center = vector(c.x, ll.y - self.track_wire * 2 + offset)
            is_too_close = any(abs(pin_added.center().x - fake_center.x) < (0.8 + self.half_wire * 4)for pin_added in self.io_pins_added_down)
            while is_too_close:         
                debug.warning("overlap, changing position") 
                add_distance = add_distance + 0.8 + self.half_wire * 4
                fake_center = vector(c.x + add_distance, ll.y - self.track_wire * 2 + offset)
                is_too_close = any(abs(pin_added.center().x - fake_center.x) < (0.8 + self.half_wire * 4)for pin_added in self.io_pins_added_down)   
            
        if edge == "right":
            fake_center = vector(ur.x + self.track_wire * 2 - offset, c.y)
            is_too_close = any(abs(pin_added.center().y - fake_center.y) < (0.8 + self.half_wire * 4)for pin_added in self.io_pins_added_right)
            while is_too_close:         
                debug.warning("overlap, changing position") 
                add_distance = add_distance + 0.8 + self.half_wire * 4
                fake_center = vector(ur.x + self.track_wire * 2 - offset, c.y + add_distance)
                # debug
                for pin_added in self.io_pins_added_right:
                    dis = abs(pin_added.center().y - fake_center.y)
                    debug.warning("current position is {0}".format(fake_center))
                    debug.warning("distance from {0} is {1}".format(pin_added, dis))
                    debug.warning("must disrance is {0}".format(0.8 + self.half_wire * 4))
                is_too_close = any(abs(pin_added.center().y - fake_center.y) < (0.8 + self.half_wire * 4)for pin_added in self.io_pins_added_right)   
            
        if edge == "top":
            fake_center = vector(c.x, ur.y + self.track_wire * 2 - offset)
            is_too_close = any(abs(pin_added.center().x - fake_center.x) < (0.8 + self.half_wire * 4)for pin_added in self.io_pins_added_up)
            while is_too_close:         
                debug.warning("overlap, changing position")
                add_distance = add_distance + 0.8 + self.half_wire * 4 
                fake_center = vector(c.x + add_distance, ur.y + self.track_wire * 2 - offset)
                is_too_close = any(abs(pin_added.center().x - fake_center.x) < (0.8 + self.half_wire * 4)for pin_added in self.io_pins_added_up)  
                    
        # Create the fake pin shape, here make sure the pin in the gds will be big enough
        layer = self.get_layer(int(not vertical))
        half_wire_vector = vector([self.half_wire] * 2)
        nll = fake_center - half_wire_vector - half_wire_vector 
        nur = fake_center + half_wire_vector + half_wire_vector
        
        rect = [nll, nur]
        pin = graph_shape(name=pin.name + "_" + "fake",
                          rect=rect,
                          layer_name_pp=layer)
        print("this create_fake_pin")
        print(pin.name)
        print(pin.center)

        if edge == "left":
            self.io_pins_added_left.append(pin)
        elif edge == "bottom":
            self.io_pins_added_down.append(pin)
        elif edge == "right":
            self.io_pins_added_right.append(pin)
        elif edge == "top":
            self.io_pins_added_up.append(pin)
        debug.warning("pin added: {0}".format(pin))
        
        return pin


    def add_io_pins(self, pin_names):
        """ Add IO pins on the edges without routing them. """
        debug.info(1, "Adding IO pins on the perimeter...")

        # Prepare GDS reader (if necessary for pin/blockage identification)
        self.prepare_gds_reader()

        # Find pins to be added (without routing)
        for name in pin_names:
            self.find_pins(name)# this will add the pins to the self.pins

        # Replace layout pins with those on the perimeter
        for name in self.pins:
            pin = next(iter(self.pins[name]))
            fake_pin = self.create_fake_pin(pin)
            self.design.replace_layout_pin(name, fake_pin) # if do not have replace_layout_pin, final gds won't have io pins
   
   
    def remove_io_pins(self, pin_name):
        # remove io pin in gds, so we could reroute
        self.design.remove_layout_pin(pin_name)





