#Armando Alvarez
#CSC501-HW2
#3/18/25
#SENSOR CONNECTIVITY USING BFS AND DFS

import random
import math
from collections import deque
import heapq

eamp = 0.1			#Eamp = 100 pJ/bit/m2 = 0.1 nJ/bit/m2 calculates the energy consumption on the transmit amplifier for one bit
eelec = 100.0		# Eelec = 100 nJ/bit is the energy consumption per bit on the transmitter circuit and receiver circuit


def random_nodes(N, x, y):	#Generates a dictionary of random, non-repeating (x, y) nodes
	node_hsh = {}
	used = set()
	while len(node_hsh) < N:
		rand_x = random.randint(0, x)
		rand_y = random.randint(0, y)
		if (rand_x, rand_y) not in used:
			index = len(node_hsh) + 1
			node_hsh[index] = (rand_x, rand_y)
			used.add((rand_x, rand_y))
	return node_hsh

def adjacency(node_hsh, Tr):	#Generates both an adjacency list and matrix for a dictionary of nodes sensing Tr distance
	node_length = len(node_hsh)
	node_list = list(node_hsh.items())
	adjacency_list = {i: [] for i in node_hsh}
	adjacency_matrix = {i: {j: 0 for j in node_hsh} for i in node_hsh}
	for i in range(node_length):
		for j in range(i+1, node_length):
			nodecenter, (x1, y1) = node_list[i]
			noderadius, (x2, y2) = node_list[j]
			distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
			if distance <= Tr:
				adjacency_list[nodecenter].append((noderadius, distance))
				adjacency_list[noderadius].append((nodecenter, distance))
				adjacency_matrix[nodecenter][noderadius] = distance
				adjacency_matrix[noderadius][nodecenter] = distance
	return adjacency_matrix, adjacency_list

def bfs_matrix(adj_matrix):	#Creates a list of connected nodes Tr distance apart using breadth first search on a matrix
	discovered = set()
	connected_components = []
	count = 0
	for i in adj_matrix:	#Start search for each node (row) of the matrix that has not been discovered
		if i not in discovered:
			connected = [(i, None)]
			discovered.add(i)
			q = deque([i])
			while q:		#Removes front of queue and adds node to connected nodes list until the queue is empty
				j = q.popleft()
				for k, distance in adj_matrix[j].items():		#Adds undiscovered neighbors to the queue until there are none left
					if distance > 0 and k not in discovered:
						discovered.add(k)
						q.append(k)
						connected.append((k, distance))
			connected_components.append(connected)	#Adds the list of connected nodes to the connected components list
			count += 1
	return connected_components, count

def bfs_list(adj_list):		#Creates a list of connected nodes Tr distance apart using breadth first search on a list
	discovered = set()
	connected_components = []
	count = 0
	for i in adj_list:		#Start search for each node of the list that has not been discovered
		if i not in discovered:
			connected = [(i, None)]
			discovered.add(i)
			q = deque([(i, None)])
			while q:		#Removes front of queue and adds node to connected nodes list until the queue is empty
				front, _ = q.popleft()
				#connected.append(j)
				for neighbor, distance in adj_list[front]:	#Adds undiscovered neighbors to the queue until there are none left
					if neighbor not in discovered:
						discovered.add(neighbor)
						q.append((neighbor, distance))
						connected.append((neighbor, distance))
			connected_components.append(connected) 	#Adds the list of connected nodes to the connected components list
			count += 1
	return connected_components, count

def dfs_matrix(adj_matrix):	#Creates a list of connected nodes Tr distance apart using depth first search on a matrix
	
	explored = set()
	connected_components = []
	count = 0
	for i in adj_list:		#Start search for each node (row) of the matrix that has not been explored
		if i not in explored:
			discovered = set()
			connected = [(i, None)]
			discovered.add(i)
			stack = [(i, None)]
			while stack:		#peek top of stack and set undiscovered_neighbor false
				top, _ = stack[-1]
				undiscovered_neighbor = False
				for neighbor, distance in adj_matrix[top].items():		#push undiscovered neighbor to stack, mark as discovered, and set undiscovered_neighbor true. Break loop.
					if distance > 0 and neighbor not in discovered:
						discovered.add(neighbor)
						stack.append((neighbor, distance))
						connected.append((neighbor, distance))
						undiscovered_neighbor = True
						break
				if not undiscovered_neighbor:		#When TOS has no undiscovered neighbors: add to explored, pop TOS, and add TOS to list of connected nodes
					explored.add(top)
					stack.pop()
			connected_components.append(connected) #When stack is empty, add list of connected nodes to list of connected components
			count += 1
	return connected_components, count


def dfs_list(adj_list):		#Creates a list of connected nodes Tr distance apart using depth first search on a list
	
	explored = set()
	connected_components = []
	count = 0
	for i in adj_list:		#Start search for each node of the list that has not been explored
		if i not in explored:
			discovered = set()
			connected = [(i, None)]
			discovered.add(i)
			stack = [(i, None)]
			while stack:		#peek top of stack and set undiscovered_neighbor false
				top, _ = stack[-1]
				undiscovered_neighbor = False
				for neighbor, distance in adj_list[top]:		#push undiscovered neighbor to stack, mark as discovered, and set undiscovered_neighbor true. Break loop.
					if neighbor not in discovered:
						discovered.add(neighbor)
						stack.append((neighbor, distance))
						connected.append((neighbor, distance))
						undiscovered_neighbor = True
						break
				if not undiscovered_neighbor:		#When TOS has no undiscovered neighbors: add to explored, pop TOS, and add TOS to list of connected nodes
					explored.add(top)
					stack.pop()
			connected_components.append(connected)		#When stack is empty, add list of connected nodes to list of connected components
			count += 1
	return connected_components, count

def shortest_paths_list(connected_components, adjacency_list, data):		#Finds the lowest energy path from a random node to all nodes of connected components using an adjacency list Dijkstra's algorithm
	short_paths = []
	total_energy = 0
	min_data = data[0]
	max_data = data[1]
	data_packets = {}
	weighted_edges = {}
	graph_edges = []

	for connections in connected_components:			#random value for the number of packets at each node based on min max user input
		if len(connections) > 1:
			for i, _ in connections:
				if i not in data_packets:
					data_packets[i] = random.randint(min_data, max_data)

	for connections in connected_components:			#Calculates the energy weight of each edge
		if len(connections) <= 1:
			continue
		path_edges = []
		path_nodes = [i for i, _ in connections]
		for n1 in data_packets:
			for n2, weight in adjacency_list[n1]:
				if n1 < n2 and n2 in path_nodes:
					curr_data = data_packets[n1]
					energy = (eelec * curr_data + eamp * curr_data * weight**2) + (eelec * curr_data)
					weighted_edges[(n1, n2)] = energy
					weighted_edges[(n2, n1)] = energy
					path_edges.append((n1, n2, energy))
		graph_edges.append(path_edges)

	for connections in connected_components:			#creates a priority queue starting at a random node and sets that distance = 0
		if len(connections) <= 1:
			continue
		ri = random.choice([i for i, _ in connections])
		distances = {i: float("inf") for i, _ in connections}
		distances[ri] = 0
		pq = [(0, ri)]
		heapq.heapify(pq)
		S = set()
		path_energy = 0
		while pq:												#Dijkstra's algorithm: While queue has nodes, repeatedly take the next in queue (smallest) and for each neighbor, 
			current_distance, current_node = heapq.heappop(pq)  #check if it provides a way for the shortest path, if so, update distance and push onto queue
			if current_node in S:
				continue
			S.add(current_node)
			for neighbor, _ in adjacency_list[current_node]:
				if neighbor in distances:
					energy_weight = weighted_edges.get((current_node, neighbor)) or weighted_edges.get((neighbor, current_node))
					if energy_weight is None:
						continue
					tentative_low = current_distance + energy_weight
					if tentative_low < distances[neighbor]:
						distances[neighbor] = tentative_low
						heapq.heappush(pq, (tentative_low, neighbor))
		path_energy = sum(									#Sum "distances" for a shortest path or energy consumption for that path.
			dist for node, dist in distances.items()
			if node != ri
		)
		total_energy += path_energy							#Sum all shortest paths or path energy consumption for a total network energy consumption
		short_paths.append({"Rendezvous": ri, "Distances": distances, "Energy": path_energy})
	return short_paths, total_energy, graph_edges						#return each shortest path, total energy consumption, and list of weighted edges

def shortest_paths_matrix(connected_components, adjacency_matrix, data):		#Same as shortest_paths_list but uses an adjacency_matrix
	short_paths = []
	total_energy = 0
	min_data = data[0]
	max_data = data[1]
	data_packets = {}
	weighted_edges = {}
	graph_edges = []

	for connections in connected_components:
		if len(connections) > 1:
			for i, _ in connections:
				if i not in data_packets:
					data_packets[i] = random.randint(min_data, max_data)
	for connections in connected_components:
		if len(connections) <= 1:
			continue
		path_edges = []
		path_nodes = [i for i, _ in connections]
		for n1 in path_nodes:
			for n2, weight in adjacency_matrix[n1].items():
				if n1 < n2 and n2 in path_nodes and weight > 0:
					curr_data = data_packets[n1]
					energy = (eelec * curr_data + eamp * curr_data * weight**2) + (eelec * curr_data)
					weighted_edges[(n1, n2)] = energy
					weighted_edges[(n2, n1)] = energy
					path_edges.append((n1, n2, energy))
		graph_edges.append(path_edges)

	for connections in connected_components:
		if len(connections) <= 1:
			continue
		ri = random.choice([i for i, _ in connections])
		distances = {i: float("inf") for i, _ in connections}
		distances[ri] = 0
		pq = [(0, ri)]
		heapq.heapify(pq)
		S = set()
		path_energy = 0
		while pq:
			current_distance, current_node = heapq.heappop(pq)
			if current_node in S:
				continue
			S.add(current_node)
			for neighbor, _ in adjacency_matrix[current_node].items():
				if neighbor in distances:
					energy_weight = weighted_edges.get((current_node, neighbor)) or weighted_edges.get((neighbor, current_node))
					if energy_weight is None:
						continue
					tentative_low = current_distance + energy_weight
					if tentative_low < distances[neighbor]:
						distances[neighbor] = tentative_low
						heapq.heappush(pq, (tentative_low, neighbor))
		path_energy = sum(
			dist for node, dist in distances.items()
			if node != ri
		)
		total_energy += path_energy
		short_paths.append({"Rendezvous": ri, "Distances": distances, "Energy": path_energy})
	return short_paths, total_energy, graph_edges

def minimum_spanning_trees(graph_edges, short_paths):			#Finds the lowest energy mst starting from a random node to other nodes
	mst_paths = []
	total_mst_energy = 0
	for i, weighted_edges in enumerate(graph_edges):

		graph = {}									#Makes adjacency list from weighted edges
		for u, v, weight in weighted_edges:
			graph.setdefault(u, []).append((v, weight))
			graph.setdefault(v, []).append((u, weight))

		ri = short_paths[i]["Rendezvous"]
		visited = {ri}
		mst = []
		pq = [(weight, ri, neighbor) for neighbor, weight in graph[ri]]		#Initialize priority queue with all edges starting with random node
		heapq.heapify(pq)

		while pq:			#while queue has nodes, get next lowest edge
			weight, u, v = heapq.heappop(pq)
			if v not in visited:		#If recieving node has not been visited
				visited.add(v)
				mst.append((u, v, weight))		#Mark visited and add to mst
				for neighbor, neighbor_weight in graph[v]:		#Add neighbors to queue
					heapq.heappush(pq, (neighbor_weight, v, neighbor))

		energy = sum(e for _, _, e in mst)		#sum mst edges for path energy
		total_mst_energy += energy			#sum path energies for total energy
		mst_paths.append({"Rendezvous": ri, "Energy": energy, "MST": mst})
	return mst_paths, total_mst_energy			#return paths and energy consumptions


while True:
	n = int(input("Enter the number of sensor nodes: "))
	x = int(input("Enter the x axis: "))
	y = int(input("Enter the y axis: "))
	nodes = random_nodes(n, x, y)
	t = int(input("Enter the transmission range: "))
	adj_matrix, adj_list = adjacency(nodes, t)
	gs = (input("Enter a for adjacency matrix or b for adjacency list: "))
	tm = (input("Enter a for BFS or b for DFS: "))
	data = tuple(map(int, input("Enter minimum and maximum SEPARATED BY A SPACE for data range: ").split()))
	break

#print(adj_matrix)

if gs == 'a':
	if tm == 'a':
		print("\nBFS Matrix Connectivity")
		conn_comps, count = bfs_matrix(adj_matrix)
		sp, tot_e, graph = shortest_paths_matrix(conn_comps, adj_matrix, data)
		mst, mst_tot_e = minimum_spanning_trees(graph, sp)
	elif tm == 'b':
		print("\nDFS Matrix Connectivity")
		conn_comps, count = dfs_matrix(adj_matrix)
		sp, tot_e, graph = shortest_paths_matrix(conn_comps, adj_matrix, data)
		mst, mst_tot_e = minimum_spanning_trees(graph, sp)
	else:
		print("Invalid traversal type")
elif gs == 'b':
	if tm == 'a':
		print("\nBFS List Connectivity")
		conn_comps, count = bfs_list(adj_list)
		sp, tot_e, graph = shortest_paths_list(conn_comps, adj_list, data)
		mst, mst_tot_e = minimum_spanning_trees(graph, sp)
	elif tm == 'b':
		print("\nDFS List Connectivity")
		conn_comps, count = dfs_list(adj_list)
		sp, tot_e, graph = shortest_paths_list(conn_comps, adj_list, data)
		mst, mst_tot_e = minimum_spanning_trees(graph, sp)
	else:
		Print("Invalid traversal type")
else:
	print("Invalid Graph Type")

print("There are ", count, " connected components\n")
for i in conn_comps:
	print(i)
	print("\n")

print("\nShortest Path Energy Consumption")
for j in sp:
	print(f"\nRendezvous point: {j['Rendezvous']}")
	for node, distance in j["Distances"].items():
		print(f" Node {node} Energy: {distance:.2f}")
	print(f"Energy Consumption: {j['Energy']}")
print(f"\nShortest Path Total Network Energy Consumption: {tot_e:.2f}")

print("\n\nMinimum Spanning Tree Energy Consumption")
for i in mst:
    print(f"\nRendezvous point: {i['Rendezvous']}")
    for u, v, w in i["MST"]:
        print(f"  Edge {u} — {v}, Energy: {w:.2f}")
    print(f"  MST Energy: {i['Energy']:.2f}")

print(f"\nMST Total Network Energy Consumption: {mst_tot_e:.2f}")
