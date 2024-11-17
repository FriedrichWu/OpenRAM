# See LICENSE for licensing information.
#
# Copyright (c) 2016-2024 Regents of the University of California, Santa Cruz
# All rights reserved.
#
from openram import debug
from openram.base.vector import vector
from openram import OPTS
from .graph import graph
from .graph_shape import graph_shape
from .router import router

class supply_placer(router):

    def __init__(self, layers, design, bbox=None, pin_type=None, ext_vdd_name="vccd1", ext_gnd_name="vssd1"):

        # `router` is the base router class
        router.__init__(self, layers, design, bbox)

        # Side supply pin type
        # (can be "top", "bottom", "right", "left", and "ring")
        self.pin_type = pin_type
        # New pins are the side supply pins
        self.new_pins = {}
        # external power name of the whole macro
        self.ext_vdd_name = ext_vdd_name
        self.ext_gnd_name = ext_gnd_name
        # instances
        self.insts = self.design.insts
        # moat pins
        self.moat_pins = []
        # io pins
        self.io_pins_left = []
        self.io_pins_right = []
        self.io_pins_top = []
        self.io_pins_bottom = []

    def route_bank(self, vdd_name="vdd", gnd_name="gnd"):
        debug.info(1, "Running router for {} and {}...".format(vdd_name, gnd_name))

        # Save pin names
        self.vdd_name = vdd_name
        self.gnd_name = gnd_name

        # Prepare gdsMill to find pins and blockages
        self.prepare_gds_reader()

        # Find vdd/gnd pins of bank, to be routed
        self.moat_pins = self.find_pins_inside(vdd_name)
        self.find_pins_inside(gnd_name)

        # Find blockages and vias
        self.find_blockages()
        self.find_vias()

        # Convert blockages and vias if they overlap a pin
        self.convert_vias()
        self.convert_blockages()

        # Add vdd and gnd pins as blockages as well
        # NOTE: This is done to make vdd and gnd pins DRC-safe
        for pin in self.all_pins:
            self.blockages.append(self.inflate_shape(pin))

        # Route vdd and gnd
        routed_count = 0
        routed_max = len(self.pins[vdd_name]) + len(self.pins[gnd_name])
        for pin_name in [vdd_name, gnd_name]:
            pins = self.pins[pin_name]
            # Route closest pins according to the minimum spanning tree
            for source, target in self.get_mst_pairs(list(pins)):
                # Create the graph
                g = graph(self)
                g.create_graph(source, target)
                # Find the shortest path from source to target
                path = g.find_shortest_path()
                # If no path is found, throw an error
                if path is None:
                    self.write_debug_gds(gds_name="{}error.gds".format(OPTS.openram_temp), g=g, source=source, target=target)
                    debug.error("Couldn't route from {} to {}.".format(source, target), -1)
                # Create the path shapes on layout
                new_wires, new_vias = self.add_path(path)
                # Find the recently added shapes
                self.find_blockages(pin_name, new_wires)
                self.find_vias(new_vias)
                # Report routed count
                routed_count += 1
                debug.info(2, "Routed {} of {} supply pins".format(routed_count, routed_max))


    #def route_moat(self):
        
    #def route_other(self):

    def check_overlap(self, moat_pin, io_pin_names): 
        # use all the IO pins(at correspoding edge) to check overlap, check 1 moat vdd pin, give the corresponding target/source position as list, and pull the source up to m3  
        add_distance = 0
        direction = 1
        self.prepare_io_pins(io_pin_names)
        # judge the edge of moat vdd
        edge = self.get_closest_edge(moat_pin)
        source_center = moat_pin.center()
        if edge == "bottom":
            pin_too_close = any(abs(io_pin.center().x - source_center.x) < self.track_width for io_pin in self.io_pins_bottom)
            tmp_center = source_center
            while pin_too_close:
                tmp_center = source_center
                add_distance = add_distance + 0.1
                if direction == 1: # right shift
                    tmp_center = tmp_center + add_distance 
                else: # left shift
                    tmp_center = tmp_center - add_distance
                pin_too_close = any(abs(io_pin.center().x - tmp_center.x) < self.track_width for io_pin in self.io_pins_bottom)
            if tmp_center == source_center: # no overlap 
                # no jog, direct pull to m3
                self.design.copy_power_pin(moat_pin, loc=None, directions=None, new_name="")
            else: # need jog
                # shift the center
                # add rectangle at same layer (original)
                self.design.add
        elif edge == "top":
            pass
        elif edge == "left":
            pass
        else: #right
            pass

    def prepare_io_pins(self, io_pin_names):
        # io_pin_names is a list
        # find all the io pins
        for pin_name in io_pin_names:
            self.find_pins(pin_name)# pin now in self.pins
            io_pin = self.pins[pin_name]
            self.find_closest_edge(io_pin)


    def get_closest_edge(self, pin):
        """ Return a point's the closest edge and the edge's axis direction. Here we use to find the edge of moat vdd """

        ll, ur = self.bbox
        point = pin.center()
        # Snap the pin to the perimeter and break the iteration
        ll_diff_x = abs(point.x - ll.x)
        ll_diff_y = abs(point.y - ll.y)
        ur_diff_x = abs(point.x - ur.x)
        ur_diff_y = abs(point.y - ur.y)
        min_diff = min(ll_diff_x, ll_diff_y, ur_diff_x, ur_diff_y)

        if min_diff == ll_diff_x:
            return "left"
        if min_diff == ll_diff_y:
            return "bottom"
        if min_diff == ur_diff_x:
            return "right"
        return "top"

       
    def find_closest_edge(self, pin):
        """ Use to find the edge, where the io pin locats """

        ll, ur = self.bbox
        point = pin.center()
        # Snap the pin to the perimeter and break the iteration
        ll_diff_x = abs(point.x - ll.x)
        ll_diff_y = abs(point.y - ll.y)
        ur_diff_x = abs(point.x - ur.x)
        ur_diff_y = abs(point.y - ur.y)
        min_diff = min(ll_diff_x, ll_diff_y, ur_diff_x, ur_diff_y)

        if min_diff == ll_diff_x:
            self.io_pins_left.append(pin)
        elif min_diff == ll_diff_y:
            self.io_pins_bottom.append(pin)
        elif min_diff == ur_diff_x:
            self.io_pins_right.append(pin)
        else:
            self.io_pins_top.append(pin)        


    def add_side_pin(self, pin_name, side, num_vias=3, num_fake_pins=4):
        """ Add supply pin to one side of the layout. """

        ll, ur = self.bbox
        vertical = side in ["left", "right"]
        inner = pin_name == self.ext_vdd_name

        # Calculate wires' wideness
        wideness = self.track_wire * num_vias + self.track_space * (num_vias - 1)

        # Calculate the offset for the inner ring
        if inner:
            margin = wideness * 2
        else:
            margin = 0

        # Calculate the lower left coordinate
        if side == "top":
            offset = vector(ll.x + margin, ur.y - wideness - margin)
        elif side == "bottom":
            offset = vector(ll.x + margin, ll.y + margin)
        elif side == "left":
            offset = vector(ll.x + margin, ll.y + margin)
        elif side == "right":
            offset = vector(ur.x - wideness - margin, ll.y + margin)

        # Calculate width and height
        shape = ur - ll
        if vertical:
            shape_width = wideness
            shape_height = shape.y
        else:
            shape_width = shape.x
            shape_height = wideness
        if inner:
            if vertical:
                shape_height -= margin * 2
            else:
                shape_width -= margin * 2

        # Add this new pin
        layer = self.get_layer(int(vertical))
        pin = self.design.add_layout_pin(text=pin_name,
                                         layer=layer,
                                         offset=offset,
                                         width=shape_width,
                                         height=shape_height)

        # Add fake pins on this new pin evenly
        fake_pins = []
        if vertical:
            space = (shape_height - (2 * wideness) - num_fake_pins * self.track_wire) / (num_fake_pins + 1)
            start_offset = vector(offset.x, offset.y + wideness)
        else:
            space = (shape_width - (2 * wideness) - num_fake_pins * self.track_wire) / (num_fake_pins + 1)
            start_offset = vector(offset.x + wideness, offset.y)
        for i in range(1, num_fake_pins + 1):
            if vertical:
                offset = vector(start_offset.x, start_offset.y + i * (space + self.track_wire))
                ll = vector(offset.x, offset.y - self.track_wire)
                ur = vector(offset.x + wideness, offset.y)
            else:
                offset = vector(start_offset.x + i * (space + self.track_wire), start_offset.y)
                ll = vector(offset.x - self.track_wire, offset.y)
                ur = vector(offset.x, offset.y + wideness)
            rect = [ll, ur]
            fake_pin = graph_shape(name=pin_name,
                                   rect=rect,
                                   layer_name_pp=layer)
            fake_pins.append(fake_pin)
        return pin, fake_pins


    def add_ring_pin(self, pin_name, num_vias=3, num_fake_pins=4):
        """ Add the supply ring to the layout. """

        # Add side pins
        new_pins = []
        for side in ["top", "bottom", "right", "left"]:
            new_shape, fake_pins = self.add_side_pin(pin_name, side, num_vias, num_fake_pins)
            ll, ur = new_shape.rect
            rect = [ll, ur]
            layer = self.get_layer(side in ["left", "right"])
            new_pin = graph_shape(name=pin_name,
                                  rect=rect,
                                  layer_name_pp=layer)
            new_pins.append(new_pin)
            #self.pins[pin_name].update(fake_pins)
            self.fake_pins.extend(fake_pins)

        # Add vias to the corners
        shift = self.track_wire + self.track_space
        half_wide = self.track_wire / 2
        for i in range(4):
            ll, ur = new_pins[i].rect
            if i % 2:
                top_left = vector(ur.x - (num_vias - 1) * shift - half_wide, ll.y + (num_vias - 1) * shift + half_wide)
            else:
                top_left = vector(ll.x + half_wide, ur.y - half_wide)
            for j in range(num_vias):
                for k in range(num_vias):
                    offset = vector(top_left.x + j * shift, top_left.y - k * shift)
                    self.design.add_via_center(layers=self.layers,
                                               offset=offset)

        # Save side pins for routing
        self.new_pins[pin_name] = new_pins
        for pin in new_pins:
            self.blockages.append(self.inflate_shape(pin))


    def get_mst_pairs(self, pins):
        """
        Return the pin pairs from the minimum spanning tree in a graph that
        connects all pins together.
        """

        pin_count = len(pins)

        # Create an adjacency matrix that connects all pins
        edges = [[0] * pin_count for i in range(pin_count)]
        for i in range(pin_count):
            for j in range(pin_count):
                # Skip if they're the same pin
                if i == j:
                    continue
                # Skip if both pins are fake
                if pins[i] in self.fake_pins and pins[j] in self.fake_pins:
                    continue
                edges[i][j] = pins[i].distance(pins[j])

        pin_connected = [False] * pin_count
        pin_connected[0] = True

        # Add the minimum cost edge in each iteration (Prim's)
        mst_pairs = []
        for i in range(pin_count - 1):
            min_cost = float("inf")
            s = 0
            t = 0
            # Iterate over already connected pins
            for m in range(pin_count):
                # Skip if not connected
                if not pin_connected[m]:
                    continue
                # Iterate over this pin's neighbors
                for n in range(pin_count):
                    # Skip if already connected or isn't a neighbor
                    if pin_connected[n] or edges[m][n] == 0:
                        continue
                    # Choose this edge if it's better the the current one
                    if edges[m][n] < min_cost:
                        min_cost = edges[m][n]
                        s = m
                        t = n
            pin_connected[t] = True
            mst_pairs.append((pins[s], pins[t]))

        return mst_pairs


    def get_new_pins(self, name):
        """ Return the new supply pins added by this router. """

        return self.new_pins[name]