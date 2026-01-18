/*
 * NCCL Mesh Plugin - Routing and Topology Discovery
 *
 * Implements topology discovery and routing for partial mesh networks.
 * Separates high-speed RDMA fabric (100-200Gbps) from management network (10GbE).
 *
 * Key features:
 * - NIC classification by link speed (fast lane vs management)
 * - Automatic topology detection (ring, line, full mesh)
 * - BFS-based shortest path routing for non-adjacent nodes
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/stat.h>
#include <arpa/inet.h>

#include "nccl/net.h"      /* NCCL log levels */
#include "mesh_plugin.h"
#include "mesh_routing.h"

/* Global routing state */
struct mesh_routing_state g_mesh_routing = {0};

/* External reference to main plugin state */
extern struct mesh_plugin_state g_mesh_state;

/* Forward declarations for internal functions */
static int routing_bfs_shortest_path(int src_idx, int dst_idx, uint8_t *path, int *path_len);
static int count_node_fast_neighbors(int node_idx);

/*
 * =============================================================================
 * NIC Classification and Speed Detection
 * =============================================================================
 */

/*
 * Get link speed in Mbps for a network interface
 * Reads from /sys/class/net/<if_name>/speed
 *
 * Returns: Speed in Mbps, or -1 on error
 */
int mesh_get_link_speed_mbps(const char *if_name) {
    char path[256];
    char buf[32];
    int fd, n;
    int speed = -1;

    if (!if_name || !if_name[0]) {
        return -1;
    }

    snprintf(path, sizeof(path), "/sys/class/net/%s/speed", if_name);

    fd = open(path, O_RDONLY);
    if (fd < 0) {
        /* Interface might not support speed reporting */
        MESH_DEBUG("Could not read speed for %s: %s", if_name, strerror(errno));
        return -1;
    }

    n = read(fd, buf, sizeof(buf) - 1);
    close(fd);

    if (n > 0) {
        buf[n] = '\0';
        speed = atoi(buf);
        /* -1 or 0 means unknown/down */
        if (speed <= 0) {
            speed = -1;
        }
    }

    return speed;
}

/*
 * Classify a NIC as fast lane or management based on link speed
 */
enum mesh_nic_lane mesh_classify_nic_lane(int speed_mbps) {
    if (speed_mbps <= 0) {
        return MESH_LANE_UNKNOWN;
    }

    if (speed_mbps >= MESH_SPEED_FAST_LANE_MIN) {
        return MESH_LANE_FAST;
    }

    if (speed_mbps <= MESH_SPEED_MANAGEMENT_MAX) {
        return MESH_LANE_MANAGEMENT;
    }

    /* Middle ground (25-50 Gbps) - treat as fast if closer to fast threshold */
    if (speed_mbps > (MESH_SPEED_FAST_LANE_MIN + MESH_SPEED_MANAGEMENT_MAX) / 2) {
        return MESH_LANE_FAST;
    }

    return MESH_LANE_MANAGEMENT;
}

/*
 * Get lane classification for a NIC by index
 */
enum mesh_nic_lane mesh_get_nic_lane(int nic_idx) {
    if (nic_idx < 0 || nic_idx >= g_mesh_state.num_nics) {
        return MESH_LANE_UNKNOWN;
    }
    return (enum mesh_nic_lane)g_mesh_state.nics[nic_idx].lane;
}

/*
 * Check if NIC is fast lane
 */
int mesh_is_fast_lane_nic(int nic_idx) {
    return mesh_get_nic_lane(nic_idx) == MESH_LANE_FAST;
}

/*
 * Check if NIC is management lane
 */
int mesh_is_management_nic(int nic_idx) {
    return mesh_get_nic_lane(nic_idx) == MESH_LANE_MANAGEMENT;
}

/*
 * Get string name for lane type
 */
const char* mesh_lane_name(enum mesh_nic_lane lane) {
    switch (lane) {
        case MESH_LANE_UNKNOWN:    return "unknown";
        case MESH_LANE_MANAGEMENT: return "management";
        case MESH_LANE_FAST:       return "fast";
        default:                   return "invalid";
    }
}

/*
 * =============================================================================
 * Node ID Generation
 * =============================================================================
 */

/*
 * Generate unique node ID from all local addresses
 * Uses FNV-1a hash of all fast lane IP addresses
 */
uint32_t mesh_generate_node_id(void) {
    uint32_t hash = 0x811c9dc5;  /* FNV-1a initial value */
    int found_any = 0;

    for (int i = 0; i < g_mesh_state.num_nics; i++) {
        struct mesh_nic *nic = &g_mesh_state.nics[i];

        /* Include all NICs with valid IPs in the hash */
        if (nic->ip_addr != 0) {
            hash ^= nic->ip_addr;
            hash *= 0x01000193;  /* FNV-1a prime */
            found_any = 1;
        }
    }

    /* If no IPs found, use a random-ish value based on time and pid */
    if (!found_any) {
        hash ^= (uint32_t)getpid();
        hash *= 0x01000193;
        hash ^= (uint32_t)time(NULL);
        hash *= 0x01000193;
    }

    MESH_DEBUG("Generated node ID: 0x%08x", hash);
    return hash;
}

/*
 * Build local node identity structure
 * Separates fast lane and management addresses
 */
int mesh_build_local_identity(struct mesh_node_identity *identity) {
    if (!identity) {
        return -1;
    }

    memset(identity, 0, sizeof(*identity));
    identity->magic = MESH_NODE_ID_MAGIC;
    identity->node_id = g_mesh_routing.local_node_id;

    for (int i = 0; i < g_mesh_state.num_nics; i++) {
        struct mesh_nic *nic = &g_mesh_state.nics[i];

        if (nic->ip_addr == 0) {
            continue;  /* Skip NICs without IP */
        }

        struct mesh_addr_entry entry = {
            .ip = htonl(nic->ip_addr),
            .mask = htonl(nic->netmask),
            .qp_num = 0,  /* Filled in during listen */
            .nic_idx = (uint8_t)i,
            .gid_index = (uint8_t)nic->gid_index,
        };

        if (nic->lane == MESH_LANE_FAST) {
            if (identity->num_fast_addrs < MESH_MAX_ADDRS) {
                identity->fast_addrs[identity->num_fast_addrs++] = entry;
            }
        } else if (nic->lane == MESH_LANE_MANAGEMENT) {
            if (identity->num_mgmt_addrs < MESH_MAX_ADDRS) {
                identity->mgmt_addrs[identity->num_mgmt_addrs++] = entry;
            }
        } else {
            /* Unknown lane - include in management by default */
            if (identity->num_mgmt_addrs < MESH_MAX_ADDRS) {
                identity->mgmt_addrs[identity->num_mgmt_addrs++] = entry;
            }
        }
    }

    MESH_INFO("Local identity: node_id=0x%08x, fast_addrs=%d, mgmt_addrs=%d",
              identity->node_id, identity->num_fast_addrs, identity->num_mgmt_addrs);

    return 0;
}

/*
 * =============================================================================
 * Routing Initialization and Shutdown
 * =============================================================================
 */

/*
 * Initialize routing subsystem
 * Should be called after mesh_init_nics() and before connections are made
 */
int mesh_routing_init(void) {
    MESH_INFO("Initializing routing subsystem");

    memset(&g_mesh_routing, 0, sizeof(g_mesh_routing));
    pthread_mutex_init(&g_mesh_routing.mutex, NULL);

    /* Classify local NICs by speed */
    if (mesh_classify_local_nics() != 0) {
        MESH_WARN("NIC classification failed, using defaults");
    }

    /* Generate our node ID */
    g_mesh_routing.local_node_id = mesh_generate_node_id();

    /* Build our identity */
    if (mesh_build_local_identity(&g_mesh_routing.local_identity) != 0) {
        MESH_WARN("Failed to build local identity");
        return -1;
    }

    /* Register ourselves as node 0 */
    if (mesh_register_node(&g_mesh_routing.local_identity) != 0) {
        MESH_WARN("Failed to register local node");
        return -1;
    }

    g_mesh_routing.initialized = 1;

    mesh_dump_nic_classification();

    return 0;
}

/*
 * Shutdown routing subsystem
 */
void mesh_routing_destroy(void) {
    MESH_INFO("Shutting down routing subsystem");

    pthread_mutex_lock(&g_mesh_routing.mutex);
    g_mesh_routing.initialized = 0;
    g_mesh_routing.discovery_complete = 0;
    g_mesh_routing.routes_valid = 0;
    pthread_mutex_unlock(&g_mesh_routing.mutex);

    pthread_mutex_destroy(&g_mesh_routing.mutex);
}

/*
 * Classify all local NICs by link speed
 */
int mesh_classify_local_nics(void) {
    int num_fast = 0;
    int num_mgmt = 0;

    for (int i = 0; i < g_mesh_state.num_nics; i++) {
        struct mesh_nic *nic = &g_mesh_state.nics[i];

        /* Get link speed */
        int speed = mesh_get_link_speed_mbps(nic->if_name);
        nic->link_speed_mbps = speed;

        /* Classify lane */
        nic->lane = (int)mesh_classify_nic_lane(speed);

        if (nic->lane == MESH_LANE_FAST) {
            num_fast++;
        } else if (nic->lane == MESH_LANE_MANAGEMENT) {
            num_mgmt++;
        }

        char ip_str[INET_ADDRSTRLEN];
        mesh_uint_to_ip(nic->ip_addr, ip_str, sizeof(ip_str));

        MESH_INFO("NIC %d (%s/%s): speed=%d Mbps, lane=%s, ip=%s",
                  i, nic->dev_name, nic->if_name,
                  speed > 0 ? speed : 0,
                  mesh_lane_name((enum mesh_nic_lane)nic->lane),
                  ip_str);
    }

    g_mesh_routing.num_fast_nics = num_fast;
    g_mesh_routing.num_mgmt_nics = num_mgmt;

    MESH_INFO("NIC classification complete: %d fast lane, %d management",
              num_fast, num_mgmt);

    return 0;
}

/*
 * =============================================================================
 * Node Registration and Adjacency
 * =============================================================================
 */

/*
 * Register a discovered node
 * Called when we receive a node identity during handshake
 */
int mesh_register_node(const struct mesh_node_identity *identity) {
    if (!identity || identity->magic != MESH_NODE_ID_MAGIC) {
        MESH_WARN("Invalid node identity");
        return -1;
    }

    pthread_mutex_lock(&g_mesh_routing.mutex);

    /* Check if already registered */
    for (int i = 0; i < g_mesh_routing.num_nodes; i++) {
        if (g_mesh_routing.nodes[i].node_id == identity->node_id) {
            /* Already known - update identity if needed */
            memcpy(&g_mesh_routing.nodes[i], identity, sizeof(*identity));
            pthread_mutex_unlock(&g_mesh_routing.mutex);
            MESH_DEBUG("Updated known node 0x%08x", identity->node_id);
            return 0;
        }
    }

    /* Add new node */
    if (g_mesh_routing.num_nodes >= MESH_MAX_NODES) {
        pthread_mutex_unlock(&g_mesh_routing.mutex);
        MESH_WARN("Maximum nodes reached, cannot register 0x%08x", identity->node_id);
        return -1;
    }

    int idx = g_mesh_routing.num_nodes++;
    memcpy(&g_mesh_routing.nodes[idx], identity, sizeof(*identity));

    pthread_mutex_unlock(&g_mesh_routing.mutex);

    MESH_INFO("Registered node %d: id=0x%08x, fast_addrs=%d, mgmt_addrs=%d",
              idx, identity->node_id, identity->num_fast_addrs, identity->num_mgmt_addrs);

    return 0;
}

/*
 * Check if a node is already known
 */
int mesh_is_node_known(uint32_t node_id) {
    pthread_mutex_lock(&g_mesh_routing.mutex);

    for (int i = 0; i < g_mesh_routing.num_nodes; i++) {
        if (g_mesh_routing.nodes[i].node_id == node_id) {
            pthread_mutex_unlock(&g_mesh_routing.mutex);
            return 1;
        }
    }

    pthread_mutex_unlock(&g_mesh_routing.mutex);
    return 0;
}

/*
 * Get node index by ID
 * Returns -1 if not found
 */
int mesh_get_node_index(uint32_t node_id) {
    pthread_mutex_lock(&g_mesh_routing.mutex);

    for (int i = 0; i < g_mesh_routing.num_nodes; i++) {
        if (g_mesh_routing.nodes[i].node_id == node_id) {
            pthread_mutex_unlock(&g_mesh_routing.mutex);
            return i;
        }
    }

    pthread_mutex_unlock(&g_mesh_routing.mutex);
    return -1;
}

/*
 * Record that we discovered a directly adjacent node via fast lane
 */
int mesh_record_adjacency(uint32_t peer_node_id,
                          const struct mesh_node_identity *peer_identity,
                          int is_fast_lane,
                          uint32_t shared_subnet,
                          uint8_t local_nic_idx,
                          uint8_t remote_nic_idx) {
    pthread_mutex_lock(&g_mesh_routing.mutex);

    /* Find or create adjacency entry */
    struct mesh_adjacency *adj = NULL;
    for (int i = 0; i < g_mesh_routing.num_adjacencies; i++) {
        if (g_mesh_routing.adjacencies[i].node_id == peer_node_id) {
            adj = &g_mesh_routing.adjacencies[i];
            break;
        }
    }

    if (!adj) {
        if (g_mesh_routing.num_adjacencies >= MESH_MAX_NODES) {
            pthread_mutex_unlock(&g_mesh_routing.mutex);
            MESH_WARN("Maximum adjacencies reached");
            return -1;
        }
        adj = &g_mesh_routing.adjacencies[g_mesh_routing.num_adjacencies++];
        memset(adj, 0, sizeof(*adj));
        adj->node_id = peer_node_id;
    }

    if (is_fast_lane) {
        adj->is_fast_adjacent = 1;
        adj->shared_fast_subnet = shared_subnet;
        adj->local_fast_nic_idx = local_nic_idx;
        adj->remote_fast_nic_idx = remote_nic_idx;
        MESH_INFO("Recorded fast lane adjacency: node 0x%08x via NIC %d (subnet 0x%08x)",
                  peer_node_id, local_nic_idx, shared_subnet);
    } else {
        adj->is_mgmt_adjacent = 1;
        MESH_DEBUG("Recorded management adjacency: node 0x%08x", peer_node_id);
    }

    /* Also register the node if we have its identity */
    if (peer_identity) {
        mesh_register_node(peer_identity);
    }

    pthread_mutex_unlock(&g_mesh_routing.mutex);
    return 0;
}

/*
 * =============================================================================
 * Routing Table Computation
 * =============================================================================
 */

/*
 * Build routing table using BFS for shortest paths
 * Should be called after all nodes have been discovered
 */
int mesh_build_routing_table(void) {
    pthread_mutex_lock(&g_mesh_routing.mutex);

    MESH_INFO("Building routing table for %d nodes", g_mesh_routing.num_nodes);

    /* Find our index */
    int local_idx = -1;
    for (int i = 0; i < g_mesh_routing.num_nodes; i++) {
        if (g_mesh_routing.nodes[i].node_id == g_mesh_routing.local_node_id) {
            local_idx = i;
            break;
        }
    }

    if (local_idx < 0) {
        pthread_mutex_unlock(&g_mesh_routing.mutex);
        MESH_WARN("Local node not found in node list");
        return -1;
    }

    /* Clear existing routes */
    memset(g_mesh_routing.routes, 0, sizeof(g_mesh_routing.routes));

    /* Build route to each node */
    for (int dest_idx = 0; dest_idx < g_mesh_routing.num_nodes; dest_idx++) {
        struct mesh_route_entry *route = &g_mesh_routing.routes[dest_idx];
        struct mesh_node_identity *dest_node = &g_mesh_routing.nodes[dest_idx];

        route->dest_node_id = dest_node->node_id;
        route->dest_node_idx = (uint8_t)dest_idx;

        if (dest_idx == local_idx) {
            /* Route to self - trivial */
            route->reachable = 1;
            route->num_hops = 0;
            route->is_direct = 1;
            continue;
        }

        /* Check if directly adjacent via fast lane */
        int is_adjacent = 0;
        struct mesh_adjacency *adj = NULL;
        for (int i = 0; i < g_mesh_routing.num_adjacencies; i++) {
            if (g_mesh_routing.adjacencies[i].node_id == dest_node->node_id &&
                g_mesh_routing.adjacencies[i].is_fast_adjacent) {
                is_adjacent = 1;
                adj = &g_mesh_routing.adjacencies[i];
                break;
            }
        }

        if (is_adjacent && adj) {
            /* Direct route via fast lane */
            route->reachable = 1;
            route->num_hops = 1;
            route->is_direct = 1;
            route->local_nic_idx = adj->local_fast_nic_idx;
            route->remote_nic_idx = adj->remote_fast_nic_idx;

            /* Find the IP address to connect to */
            for (int a = 0; a < dest_node->num_fast_addrs; a++) {
                uint32_t peer_ip = ntohl(dest_node->fast_addrs[a].ip);
                uint32_t peer_mask = ntohl(dest_node->fast_addrs[a].mask);
                uint32_t peer_subnet = peer_ip & peer_mask;
                if (peer_subnet == adj->shared_fast_subnet) {
                    route->direct_ip = peer_ip;
                    break;
                }
            }

            MESH_DEBUG("Route to node %d (0x%08x): direct via NIC %d",
                       dest_idx, dest_node->node_id, route->local_nic_idx);
        } else {
            /* Need relay - find shortest path */
            uint8_t path[MESH_MAX_HOPS];
            int path_len = 0;

            if (routing_bfs_shortest_path(local_idx, dest_idx, path, &path_len) == 0 && path_len > 0) {
                route->reachable = 1;
                route->num_hops = (uint8_t)path_len;
                route->is_direct = 0;
                route->path_len = (uint8_t)path_len;

                /* Copy path */
                for (int p = 0; p < path_len && p < MESH_MAX_HOPS; p++) {
                    route->relay_path[p] = path[p];
                }

                /* Next hop is first node in path after us */
                if (path_len >= 2) {
                    int next_hop_idx = path[1];
                    struct mesh_node_identity *next_node = &g_mesh_routing.nodes[next_hop_idx];
                    route->next_hop_node_id = next_node->node_id;

                    /* Find adjacency to next hop */
                    for (int i = 0; i < g_mesh_routing.num_adjacencies; i++) {
                        if (g_mesh_routing.adjacencies[i].node_id == next_node->node_id &&
                            g_mesh_routing.adjacencies[i].is_fast_adjacent) {
                            struct mesh_adjacency *next_adj = &g_mesh_routing.adjacencies[i];
                            route->next_hop_nic_idx = next_adj->local_fast_nic_idx;

                            /* Find IP for next hop */
                            uint32_t shared = next_adj->shared_fast_subnet;
                            for (int a = 0; a < next_node->num_fast_addrs; a++) {
                                uint32_t peer_ip = ntohl(next_node->fast_addrs[a].ip);
                                uint32_t peer_mask = ntohl(next_node->fast_addrs[a].mask);
                                if ((peer_ip & peer_mask) == shared) {
                                    route->next_hop_ip = peer_ip;
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }

                MESH_DEBUG("Route to node %d (0x%08x): relay, %d hops, next_hop=0x%08x",
                           dest_idx, dest_node->node_id, path_len, route->next_hop_node_id);
            } else {
                /* No route found */
                route->reachable = 0;
                MESH_WARN("No route to node %d (0x%08x)", dest_idx, dest_node->node_id);
            }
        }
    }

    g_mesh_routing.routes_valid = 1;

    /* Detect topology */
    g_mesh_routing.fast_topology = mesh_detect_topology();

    pthread_mutex_unlock(&g_mesh_routing.mutex);

    MESH_INFO("Routing table built: topology=%s", mesh_topology_name(g_mesh_routing.fast_topology));
    mesh_dump_routing_table();

    return 0;
}

/*
 * BFS shortest path algorithm
 * Returns path from src to dst (including both endpoints)
 */
static int routing_bfs_shortest_path(int src_idx, int dst_idx, uint8_t *path, int *path_len) {
    if (src_idx == dst_idx) {
        path[0] = (uint8_t)src_idx;
        *path_len = 1;
        return 0;
    }

    int num_nodes = g_mesh_routing.num_nodes;
    if (num_nodes <= 0 || src_idx < 0 || dst_idx < 0 ||
        src_idx >= num_nodes || dst_idx >= num_nodes) {
        return -1;
    }

    /* BFS state */
    int visited[MESH_MAX_NODES] = {0};
    int parent[MESH_MAX_NODES];
    int queue[MESH_MAX_NODES];
    int head = 0, tail = 0;

    for (int i = 0; i < MESH_MAX_NODES; i++) {
        parent[i] = -1;
    }

    /* Start BFS from source */
    visited[src_idx] = 1;
    queue[tail++] = src_idx;

    while (head < tail) {
        int curr = queue[head++];

        if (curr == dst_idx) {
            /* Found destination - reconstruct path */
            int temp_path[MESH_MAX_HOPS];
            int len = 0;
            int node = dst_idx;

            while (node != -1 && len < MESH_MAX_HOPS) {
                temp_path[len++] = node;
                node = parent[node];
            }

            /* Reverse path */
            *path_len = len;
            for (int i = 0; i < len; i++) {
                path[i] = (uint8_t)temp_path[len - 1 - i];
            }

            return 0;
        }

        /* Explore neighbors (fast lane adjacencies only) */
        uint32_t curr_node_id = g_mesh_routing.nodes[curr].node_id;

        for (int i = 0; i < g_mesh_routing.num_adjacencies; i++) {
            struct mesh_adjacency *adj = &g_mesh_routing.adjacencies[i];

            /* Check if this adjacency involves current node */
            /* Note: We need to check both directions */

            /* If current node is our local node, check direct adjacencies */
            if (curr_node_id == g_mesh_routing.local_node_id) {
                /* Our adjacencies are recorded from our perspective */
                if (adj->is_fast_adjacent) {
                    int neighbor_idx = mesh_get_node_index(adj->node_id);
                    if (neighbor_idx >= 0 && !visited[neighbor_idx]) {
                        visited[neighbor_idx] = 1;
                        parent[neighbor_idx] = curr;
                        queue[tail++] = neighbor_idx;
                    }
                }
            }
        }

        /* Also check if other nodes are adjacent to current node
         * This requires checking each node's adjacency to curr */
        for (int n = 0; n < num_nodes; n++) {
            if (visited[n]) continue;

            /* Check if node n is adjacent to curr via fast lane */
            /* For now, we use a simplified model where adjacency is symmetric */
            /* In a real implementation, we'd need bidirectional adjacency info */

            /* Check if curr has an adjacency to n */
            struct mesh_node_identity *curr_node = &g_mesh_routing.nodes[curr];
            struct mesh_node_identity *other_node = &g_mesh_routing.nodes[n];

            /* Check if they share a fast lane subnet */
            for (int a = 0; a < curr_node->num_fast_addrs; a++) {
                uint32_t curr_ip = ntohl(curr_node->fast_addrs[a].ip);
                uint32_t curr_mask = ntohl(curr_node->fast_addrs[a].mask);
                uint32_t curr_subnet = curr_ip & curr_mask;

                for (int b = 0; b < other_node->num_fast_addrs; b++) {
                    uint32_t other_ip = ntohl(other_node->fast_addrs[b].ip);
                    uint32_t other_mask = ntohl(other_node->fast_addrs[b].mask);
                    uint32_t other_subnet = other_ip & other_mask;

                    if (curr_subnet == other_subnet && curr_mask == other_mask) {
                        /* Same subnet - they are adjacent */
                        if (!visited[n]) {
                            visited[n] = 1;
                            parent[n] = curr;
                            queue[tail++] = n;
                        }
                        goto next_node;
                    }
                }
            }
            next_node:;
        }
    }

    /* No path found */
    *path_len = 0;
    return -1;
}

/*
 * Get route to a destination node
 */
int mesh_get_route(uint32_t dest_node_id, struct mesh_route_entry *route) {
    if (!route) {
        return -1;
    }

    pthread_mutex_lock(&g_mesh_routing.mutex);

    if (!g_mesh_routing.routes_valid) {
        pthread_mutex_unlock(&g_mesh_routing.mutex);
        MESH_DEBUG("Routing table not yet valid");
        return -1;
    }

    for (int i = 0; i < g_mesh_routing.num_nodes; i++) {
        if (g_mesh_routing.routes[i].dest_node_id == dest_node_id) {
            memcpy(route, &g_mesh_routing.routes[i], sizeof(*route));
            pthread_mutex_unlock(&g_mesh_routing.mutex);
            return route->reachable ? 0 : -1;
        }
    }

    pthread_mutex_unlock(&g_mesh_routing.mutex);
    return -1;
}

/*
 * Check if we have a direct fast lane connection to a node
 */
int mesh_has_direct_route(uint32_t dest_node_id) {
    struct mesh_route_entry route;
    if (mesh_get_route(dest_node_id, &route) == 0) {
        return route.is_direct;
    }
    return 0;
}

/*
 * Check if we need relay to reach a node
 */
int mesh_needs_relay(uint32_t dest_node_id) {
    struct mesh_route_entry route;
    if (mesh_get_route(dest_node_id, &route) == 0) {
        return !route.is_direct;
    }
    return 1;  /* Unknown nodes need relay by default */
}

/*
 * =============================================================================
 * Topology Detection
 * =============================================================================
 */

/*
 * Count fast lane neighbors for a node
 */
static int count_node_fast_neighbors(int node_idx) {
    if (node_idx < 0 || node_idx >= g_mesh_routing.num_nodes) {
        return 0;
    }

    struct mesh_node_identity *node = &g_mesh_routing.nodes[node_idx];
    int count = 0;

    /* Count nodes that share a fast lane subnet with this node */
    for (int i = 0; i < g_mesh_routing.num_nodes; i++) {
        if (i == node_idx) continue;

        struct mesh_node_identity *other = &g_mesh_routing.nodes[i];

        /* Check for shared fast lane subnet */
        for (int a = 0; a < node->num_fast_addrs; a++) {
            uint32_t node_ip = ntohl(node->fast_addrs[a].ip);
            uint32_t node_mask = ntohl(node->fast_addrs[a].mask);
            uint32_t node_subnet = node_ip & node_mask;

            for (int b = 0; b < other->num_fast_addrs; b++) {
                uint32_t other_ip = ntohl(other->fast_addrs[b].ip);
                uint32_t other_mask = ntohl(other->fast_addrs[b].mask);
                uint32_t other_subnet = other_ip & other_mask;

                if (node_subnet == other_subnet && node_mask == other_mask) {
                    count++;
                    goto found_neighbor;
                }
            }
        }
        found_neighbor:;
    }

    return count;
}

/*
 * Detect the fast lane topology type
 */
enum mesh_topology_type mesh_detect_topology(void) {
    int num_nodes = g_mesh_routing.num_nodes;

    if (num_nodes <= 1) {
        return MESH_TOPO_UNKNOWN;
    }

    /* Count neighbor degrees */
    int degrees[MESH_MAX_NODES];
    int min_degree = num_nodes;
    int max_degree = 0;
    int degree_1_count = 0;
    int degree_2_count = 0;

    for (int i = 0; i < num_nodes; i++) {
        degrees[i] = count_node_fast_neighbors(i);
        if (degrees[i] < min_degree) min_degree = degrees[i];
        if (degrees[i] > max_degree) max_degree = degrees[i];
        if (degrees[i] == 1) degree_1_count++;
        if (degrees[i] == 2) degree_2_count++;

        MESH_DEBUG("Node %d (0x%08x): degree=%d",
                   i, g_mesh_routing.nodes[i].node_id, degrees[i]);
    }

    MESH_INFO("Topology analysis: nodes=%d, min_degree=%d, max_degree=%d",
              num_nodes, min_degree, max_degree);

    /* Classification */
    if (min_degree == max_degree && min_degree == num_nodes - 1) {
        /* Everyone connected to everyone else */
        MESH_INFO("Detected topology: FULL MESH");
        return MESH_TOPO_FULL_MESH;
    }

    if (num_nodes >= 3 && min_degree == 2 && max_degree == 2) {
        /* All nodes have exactly 2 neighbors - ring */
        MESH_INFO("Detected topology: RING");
        return MESH_TOPO_RING;
    }

    if (num_nodes >= 2 && degree_1_count == 2 && degree_2_count == num_nodes - 2) {
        /* Exactly 2 endpoints (degree 1), rest have degree 2 - line */
        MESH_INFO("Detected topology: LINE");
        return MESH_TOPO_LINE;
    }

    if (num_nodes >= 2 && degree_1_count == num_nodes - 1 && max_degree == num_nodes - 1) {
        /* One node connected to all others, others only to that node - star */
        MESH_INFO("Detected topology: STAR");
        return MESH_TOPO_STAR;
    }

    /* Mixed/partial topology */
    MESH_INFO("Detected topology: PARTIAL");
    return MESH_TOPO_PARTIAL;
}

/*
 * Get string name for topology type
 */
const char* mesh_topology_name(enum mesh_topology_type topo) {
    switch (topo) {
        case MESH_TOPO_UNKNOWN:    return "unknown";
        case MESH_TOPO_FULL_MESH:  return "full_mesh";
        case MESH_TOPO_RING:       return "ring";
        case MESH_TOPO_LINE:       return "line";
        case MESH_TOPO_STAR:       return "star";
        case MESH_TOPO_PARTIAL:    return "partial";
        default:                   return "invalid";
    }
}

/*
 * Check if topology is a ring
 */
int mesh_is_ring_topology(void) {
    return g_mesh_routing.fast_topology == MESH_TOPO_RING;
}

/*
 * Check if topology is a line
 */
int mesh_is_line_topology(void) {
    return g_mesh_routing.fast_topology == MESH_TOPO_LINE;
}

/*
 * =============================================================================
 * Enhanced NIC Lookup (Lane-Aware)
 * =============================================================================
 */

/*
 * Find fast lane NIC that can reach peer IP
 */
struct mesh_nic* mesh_find_fast_nic_for_ip(uint32_t peer_ip) {
    for (int i = 0; i < g_mesh_state.num_nics; i++) {
        struct mesh_nic *nic = &g_mesh_state.nics[i];

        /* Only consider fast lane NICs */
        if (nic->lane != MESH_LANE_FAST) {
            continue;
        }

        uint32_t peer_subnet = peer_ip & nic->netmask;
        if (peer_subnet == nic->subnet) {
            MESH_DEBUG("Found fast lane NIC %s for peer IP 0x%x",
                       nic->dev_name, peer_ip);
            return nic;
        }
    }

    return NULL;
}

/*
 * Find management NIC that can reach peer IP
 */
struct mesh_nic* mesh_find_mgmt_nic_for_ip(uint32_t peer_ip) {
    for (int i = 0; i < g_mesh_state.num_nics; i++) {
        struct mesh_nic *nic = &g_mesh_state.nics[i];

        /* Only consider management NICs */
        if (nic->lane != MESH_LANE_MANAGEMENT) {
            continue;
        }

        uint32_t peer_subnet = peer_ip & nic->netmask;
        if (peer_subnet == nic->subnet) {
            MESH_DEBUG("Found management NIC %s for peer IP 0x%x",
                       nic->dev_name, peer_ip);
            return nic;
        }
    }

    return NULL;
}

/*
 * Find any NIC that can reach peer IP (prefers fast lane)
 */
struct mesh_nic* mesh_find_any_nic_for_ip(uint32_t peer_ip) {
    /* Try fast lane first */
    struct mesh_nic *nic = mesh_find_fast_nic_for_ip(peer_ip);
    if (nic) {
        return nic;
    }

    /* Fall back to management */
    nic = mesh_find_mgmt_nic_for_ip(peer_ip);
    if (nic) {
        MESH_DEBUG("Using management NIC for peer IP 0x%x (no fast lane available)", peer_ip);
        return nic;
    }

    /* Fall back to original function for unknown NICs */
    return mesh_find_nic_for_ip(peer_ip);
}

/*
 * =============================================================================
 * Debug and Diagnostics
 * =============================================================================
 */

/*
 * Print NIC classification to log
 */
void mesh_dump_nic_classification(void) {
    MESH_INFO("=== NIC Classification ===");
    for (int i = 0; i < g_mesh_state.num_nics; i++) {
        struct mesh_nic *nic = &g_mesh_state.nics[i];
        char ip_str[INET_ADDRSTRLEN];
        mesh_uint_to_ip(nic->ip_addr, ip_str, sizeof(ip_str));

        MESH_INFO("  NIC %d: %s (%s)", i, nic->dev_name, nic->if_name);
        MESH_INFO("    IP: %s, Speed: %d Mbps, Lane: %s",
                  ip_str,
                  nic->link_speed_mbps > 0 ? nic->link_speed_mbps : 0,
                  mesh_lane_name((enum mesh_nic_lane)nic->lane));
    }
    MESH_INFO("  Summary: %d fast lane, %d management NICs",
              g_mesh_routing.num_fast_nics, g_mesh_routing.num_mgmt_nics);
}

/*
 * Print routing table to log
 */
void mesh_dump_routing_table(void) {
    pthread_mutex_lock(&g_mesh_routing.mutex);

    MESH_INFO("=== Routing Table (local node 0x%08x) ===", g_mesh_routing.local_node_id);

    if (!g_mesh_routing.routes_valid) {
        MESH_INFO("  (routing table not yet computed)");
        pthread_mutex_unlock(&g_mesh_routing.mutex);
        return;
    }

    for (int i = 0; i < g_mesh_routing.num_nodes; i++) {
        struct mesh_route_entry *route = &g_mesh_routing.routes[i];
        char dest_ip_str[INET_ADDRSTRLEN] = "N/A";

        if (route->is_direct && route->direct_ip) {
            mesh_uint_to_ip(route->direct_ip, dest_ip_str, sizeof(dest_ip_str));
        } else if (!route->is_direct && route->next_hop_ip) {
            mesh_uint_to_ip(route->next_hop_ip, dest_ip_str, sizeof(dest_ip_str));
        }

        if (route->dest_node_id == g_mesh_routing.local_node_id) {
            MESH_INFO("  Node %d: 0x%08x (self)", i, route->dest_node_id);
        } else if (route->reachable) {
            if (route->is_direct) {
                MESH_INFO("  Node %d: 0x%08x -> DIRECT via NIC %d (%s)",
                          i, route->dest_node_id, route->local_nic_idx, dest_ip_str);
            } else {
                MESH_INFO("  Node %d: 0x%08x -> RELAY %d hops, next=0x%08x (%s)",
                          i, route->dest_node_id, route->num_hops,
                          route->next_hop_node_id, dest_ip_str);
            }
        } else {
            MESH_INFO("  Node %d: 0x%08x -> UNREACHABLE", i, route->dest_node_id);
        }
    }

    pthread_mutex_unlock(&g_mesh_routing.mutex);
}

/*
 * Print topology summary to log
 */
void mesh_dump_topology(void) {
    pthread_mutex_lock(&g_mesh_routing.mutex);

    MESH_INFO("=== Topology Summary ===");
    MESH_INFO("  Local node ID: 0x%08x", g_mesh_routing.local_node_id);
    MESH_INFO("  Total nodes: %d", g_mesh_routing.num_nodes);
    MESH_INFO("  Fast lane topology: %s", mesh_topology_name(g_mesh_routing.fast_topology));
    MESH_INFO("  Fast lane NICs: %d", g_mesh_routing.num_fast_nics);
    MESH_INFO("  Management NICs: %d", g_mesh_routing.num_mgmt_nics);

    MESH_INFO("  Adjacencies (%d fast lane):", g_mesh_routing.num_adjacencies);
    for (int i = 0; i < g_mesh_routing.num_adjacencies; i++) {
        struct mesh_adjacency *adj = &g_mesh_routing.adjacencies[i];
        if (adj->is_fast_adjacent) {
            char subnet_str[INET_ADDRSTRLEN];
            mesh_uint_to_ip(adj->shared_fast_subnet, subnet_str, sizeof(subnet_str));
            MESH_INFO("    0x%08x: fast (subnet %s, local NIC %d)",
                      adj->node_id, subnet_str, adj->local_fast_nic_idx);
        }
    }

    pthread_mutex_unlock(&g_mesh_routing.mutex);
}
