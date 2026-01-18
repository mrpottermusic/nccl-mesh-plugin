/*
 * NCCL Mesh Plugin - Routing and Topology Discovery
 *
 * Supports partial mesh topologies (ring, line) with relay routing
 * for non-adjacent nodes. Separates high-speed RDMA fabric (100-200Gbps)
 * from management network (10GbE).
 */

#ifndef MESH_ROUTING_H
#define MESH_ROUTING_H

#include <stdint.h>
#include <pthread.h>
#include "mesh_plugin.h"

/*
 * Link speed classification thresholds (in Mbps)
 *
 * Fast lane: 100Gbps+ direct RDMA connections for collective operations
 * Management: 10GbE switched network for orchestration, checkpoints, etc.
 */
#define MESH_SPEED_FAST_LANE_MIN   50000    /* 50 Gbps minimum for fast lane */
#define MESH_SPEED_MANAGEMENT_MAX  25000    /* 25 Gbps maximum for management */

/*
 * NIC lane classification
 */
enum mesh_nic_lane {
    MESH_LANE_UNKNOWN = 0,      /* Speed not determined */
    MESH_LANE_MANAGEMENT,       /* 10GbE management network (switched, all-to-all) */
    MESH_LANE_FAST,             /* 100-200Gbps RDMA fabric (ring/line topology) */
};

/*
 * Topology types for the fast lane network
 * Management network is always assumed to be all-to-all via switch
 */
enum mesh_topology_type {
    MESH_TOPO_UNKNOWN = 0,      /* Not yet determined */
    MESH_TOPO_FULL_MESH,        /* All fast lane NICs directly connected */
    MESH_TOPO_RING,             /* Circular: each node has 2 fast lane neighbors */
    MESH_TOPO_LINE,             /* Linear: endpoints have 1, others have 2 neighbors */
    MESH_TOPO_STAR,             /* One central node connected to all (unlikely) */
    MESH_TOPO_PARTIAL,          /* Some direct, some need relay */
};

/*
 * Routing limits
 */
#define MESH_MAX_NODES      16      /* Maximum nodes in cluster */
#define MESH_MAX_HOPS       8       /* Maximum relay hops */
#define MESH_INVALID_NODE   0xFF    /* Invalid node index */
#define MESH_NODE_ID_MAGIC  0x4E4F4445  /* "NODE" */

/*
 * Node identity - exchanged during topology discovery
 * Contains all addresses for this node, classified by lane
 */
struct mesh_node_identity {
    uint32_t magic;                             /* MESH_NODE_ID_MAGIC */
    uint32_t node_id;                           /* Unique node identifier */
    uint8_t  num_fast_addrs;                    /* Number of fast lane addresses */
    uint8_t  num_mgmt_addrs;                    /* Number of management addresses */
    uint8_t  reserved[2];

    /* Fast lane addresses (100-200Gbps RDMA) */
    struct mesh_addr_entry fast_addrs[MESH_MAX_ADDRS];

    /* Management addresses (10GbE) - for reference, not used for NCCL traffic */
    struct mesh_addr_entry mgmt_addrs[MESH_MAX_ADDRS];
};

/*
 * Adjacency information for one peer
 */
struct mesh_adjacency {
    uint32_t node_id;               /* Peer's node ID */
    int is_fast_adjacent;           /* 1 if directly reachable via fast lane */
    int is_mgmt_adjacent;           /* 1 if reachable via management (always 1 if switch) */
    uint32_t shared_fast_subnet;    /* Fast lane subnet we share (0 if not adjacent) */
    uint8_t  local_fast_nic_idx;    /* Our NIC index for fast lane connection */
    uint8_t  remote_fast_nic_idx;   /* Their NIC index for fast lane connection */
    uint8_t  reserved[2];
};

/*
 * Routing table entry for reaching one destination
 */
struct mesh_route_entry {
    uint32_t dest_node_id;              /* Destination node ID */
    uint8_t  dest_node_idx;             /* Index in nodes array */
    uint8_t  reachable;                 /* 1 if route exists */
    uint8_t  num_hops;                  /* Number of hops (1 = direct) */
    uint8_t  is_direct;                 /* 1 if direct fast lane connection */

    /* For direct connections */
    uint32_t direct_ip;                 /* IP to connect to (fast lane) */
    uint8_t  local_nic_idx;             /* Our NIC to use */
    uint8_t  remote_nic_idx;            /* Their NIC index */
    uint8_t  reserved1[2];

    /* For relay connections */
    uint32_t next_hop_node_id;          /* Next hop node ID */
    uint32_t next_hop_ip;               /* IP to reach next hop */
    uint8_t  next_hop_nic_idx;          /* Our NIC for next hop */
    uint8_t  path_len;                  /* Number of nodes in path */
    uint8_t  relay_path[MESH_MAX_HOPS]; /* Full path (node indices) */
};

/*
 * Complete routing state for this node
 */
struct mesh_routing_state {
    /* Our identity */
    uint32_t local_node_id;                         /* Our unique node ID */
    struct mesh_node_identity local_identity;       /* Our full identity */

    /* Topology information */
    enum mesh_topology_type fast_topology;          /* Detected fast lane topology */
    int num_nodes;                                  /* Total nodes discovered */
    int num_fast_nics;                              /* Number of fast lane NICs locally */
    int num_mgmt_nics;                              /* Number of management NICs locally */

    /* Known nodes (populated during discovery) */
    struct mesh_node_identity nodes[MESH_MAX_NODES];

    /* Adjacency matrix for fast lane (who we can reach directly) */
    struct mesh_adjacency adjacencies[MESH_MAX_NODES];
    int num_adjacencies;

    /* Routing table */
    struct mesh_route_entry routes[MESH_MAX_NODES];
    int routes_valid;                               /* 1 if routing table is computed */

    /* State */
    int initialized;                                /* 1 if routing is initialized */
    int discovery_complete;                         /* 1 if topology discovery done */
    pthread_mutex_t mutex;                          /* Protects routing state */
};

/*
 * Global routing state (singleton, like g_mesh_state)
 */
extern struct mesh_routing_state g_mesh_routing;

/*
 * NIC classification and speed detection
 */

/* Get link speed in Mbps for a network interface */
int mesh_get_link_speed_mbps(const char *if_name);

/* Classify a NIC as fast lane or management based on speed */
enum mesh_nic_lane mesh_classify_nic_lane(int speed_mbps);

/* Get lane classification for a NIC by index */
enum mesh_nic_lane mesh_get_nic_lane(int nic_idx);

/* Check if NIC is fast lane */
int mesh_is_fast_lane_nic(int nic_idx);

/* Check if NIC is management lane */
int mesh_is_management_nic(int nic_idx);

/* Get string name for lane type */
const char* mesh_lane_name(enum mesh_nic_lane lane);

/*
 * Node ID generation
 */

/* Generate unique node ID from all local addresses */
uint32_t mesh_generate_node_id(void);

/* Build local node identity structure */
int mesh_build_local_identity(struct mesh_node_identity *identity);

/*
 * Topology discovery
 */

/* Initialize routing subsystem (call during mesh_init) */
int mesh_routing_init(void);

/* Shutdown routing subsystem */
void mesh_routing_destroy(void);

/* Classify all local NICs by speed */
int mesh_classify_local_nics(void);

/* Record that we discovered a directly adjacent node */
int mesh_record_adjacency(uint32_t peer_node_id,
                          const struct mesh_node_identity *peer_identity,
                          int is_fast_lane,
                          uint32_t shared_subnet,
                          uint8_t local_nic_idx,
                          uint8_t remote_nic_idx);

/* Register a discovered node (from handshake) */
int mesh_register_node(const struct mesh_node_identity *identity);

/* Check if a node is already known */
int mesh_is_node_known(uint32_t node_id);

/* Get node index by ID (-1 if not found) */
int mesh_get_node_index(uint32_t node_id);

/*
 * Routing table computation
 */

/* Build routing table after topology discovery is complete */
int mesh_build_routing_table(void);

/* Get route to a destination node */
int mesh_get_route(uint32_t dest_node_id, struct mesh_route_entry *route);

/* Check if we have a direct fast lane connection to a node */
int mesh_has_direct_route(uint32_t dest_node_id);

/* Check if we need relay to reach a node */
int mesh_needs_relay(uint32_t dest_node_id);

/*
 * Topology detection
 */

/* Detect the fast lane topology type */
enum mesh_topology_type mesh_detect_topology(void);

/* Get string name for topology type */
const char* mesh_topology_name(enum mesh_topology_type topo);

/* Check if topology is a ring */
int mesh_is_ring_topology(void);

/* Check if topology is a line */
int mesh_is_line_topology(void);

/*
 * Subnet-based NIC lookup (enhanced to respect lane classification)
 */

/* Find fast lane NIC that can reach peer IP */
struct mesh_nic* mesh_find_fast_nic_for_ip(uint32_t peer_ip);

/* Find management NIC that can reach peer IP */
struct mesh_nic* mesh_find_mgmt_nic_for_ip(uint32_t peer_ip);

/* Find any NIC that can reach peer IP (prefers fast lane) */
struct mesh_nic* mesh_find_any_nic_for_ip(uint32_t peer_ip);

/*
 * Debug and diagnostics
 */

/* Print routing table to log */
void mesh_dump_routing_table(void);

/* Print topology summary to log */
void mesh_dump_topology(void);

/* Print NIC classification to log */
void mesh_dump_nic_classification(void);

#endif /* MESH_ROUTING_H */
