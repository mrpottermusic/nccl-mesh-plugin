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

/*
 * =============================================================================
 * Phase 2: Relay Communication Layer
 * =============================================================================
 */

/*
 * Relay protocol constants
 */
#define MESH_RELAY_MAGIC        0x52454C59  /* "RELY" */
#define MESH_RELAY_MAX_SESSIONS 64          /* Max concurrent relay sessions */
#define MESH_RELAY_BUFFER_SIZE  (4 * 1024 * 1024)  /* 4MB relay buffer */
#define MESH_RELAY_QUEUE_SIZE   32          /* Pending relay requests */

/*
 * Relay message types
 */
enum mesh_relay_msg_type {
    MESH_RELAY_DATA = 1,        /* Data payload to forward */
    MESH_RELAY_SETUP_REQ,       /* Request to establish relay path */
    MESH_RELAY_SETUP_ACK,       /* Acknowledge relay path setup */
    MESH_RELAY_TEARDOWN,        /* Tear down relay session */
    MESH_RELAY_KEEPALIVE,       /* Keep session alive */
};

/*
 * Relay header - prepended to all relayed messages
 * This header travels with the message through all relay hops
 */
struct mesh_relay_header {
    uint32_t magic;             /* MESH_RELAY_MAGIC */
    uint32_t session_id;        /* Unique session identifier */
    uint32_t src_node_id;       /* Original sender node ID */
    uint32_t dst_node_id;       /* Final destination node ID */
    uint32_t payload_size;      /* Size of actual data (excluding header) */
    uint16_t msg_type;          /* enum mesh_relay_msg_type */
    uint8_t  hop_count;         /* Current hop number (incremented at each relay) */
    uint8_t  total_hops;        /* Total expected hops */
    uint8_t  path[MESH_MAX_HOPS]; /* Node indices in path */
    uint8_t  flags;             /* Reserved for future use */
    uint8_t  reserved[3];
};

/*
 * Relay session state
 */
enum mesh_relay_session_state {
    MESH_RELAY_STATE_IDLE = 0,      /* Session slot not in use */
    MESH_RELAY_STATE_SETUP,         /* Setting up relay path */
    MESH_RELAY_STATE_ACTIVE,        /* Session is active */
    MESH_RELAY_STATE_TEARDOWN,      /* Tearing down */
    MESH_RELAY_STATE_ERROR,         /* Error state */
};

/*
 * Relay session - represents one relay path between non-adjacent nodes
 *
 * For the ORIGINATOR (sender):
 *   - Sends data to first hop (next_hop connection)
 *   - Receives acknowledgments
 *
 * For RELAY NODES (intermediate):
 *   - Receives from prev_hop, forwards to next_hop
 *   - Maintains buffer for store-and-forward
 *
 * For the DESTINATION (receiver):
 *   - Receives data from prev_hop
 *   - Delivers to application
 */
struct mesh_relay_session {
    /* Session identification */
    uint32_t session_id;            /* Unique session ID */
    uint32_t src_node_id;           /* Original sender */
    uint32_t dst_node_id;           /* Final destination */
    enum mesh_relay_session_state state;

    /* Path information */
    uint8_t  path[MESH_MAX_HOPS];   /* Full path (node indices) */
    uint8_t  path_len;              /* Number of hops */
    uint8_t  my_position;           /* Our position in path (0=src, path_len-1=dst) */
    uint8_t  reserved[2];

    /* Connections to neighbors in path */
    void *prev_hop_comm;            /* Receive from previous hop (NULL if we're source) */
    void *next_hop_comm;            /* Send to next hop (NULL if we're destination) */
    uint32_t prev_hop_node_id;      /* Previous hop node ID */
    uint32_t next_hop_node_id;      /* Next hop node ID */

    /* Store-and-forward buffer */
    void *relay_buffer;             /* Buffer for receiving/forwarding data */
    size_t buffer_size;             /* Allocated buffer size */
    size_t data_in_buffer;          /* Current data in buffer */

    /* Statistics */
    uint64_t bytes_relayed;         /* Total bytes forwarded */
    uint64_t messages_relayed;      /* Total messages forwarded */

    /* Timing */
    uint64_t created_time;          /* When session was created */
    uint64_t last_activity;         /* Last data transfer time */
};

/*
 * Relay service state - manages all relay operations
 */
struct mesh_relay_state {
    /* Sessions */
    struct mesh_relay_session sessions[MESH_RELAY_MAX_SESSIONS];
    int num_active_sessions;
    uint32_t next_session_id;       /* Counter for generating session IDs */
    pthread_mutex_t sessions_mutex;

    /* Relay service thread */
    pthread_t relay_thread;
    int thread_running;
    int thread_stop;
    pthread_cond_t relay_cond;      /* Signals work available */

    /* Configuration */
    int relay_enabled;              /* 1 if relay routing is enabled */
    int max_relay_hops;             /* Maximum allowed relay hops */

    /* Statistics */
    uint64_t total_bytes_relayed;
    uint64_t total_messages_relayed;
    uint64_t relay_errors;
};

/*
 * Global relay state
 */
extern struct mesh_relay_state g_mesh_relay;

/*
 * Relay communication handle
 * Used by mesh_send_comm/mesh_recv_comm for relayed connections
 */
struct mesh_relay_comm {
    int is_relay;                   /* 1 if this is a relay connection */
    uint32_t session_id;            /* Associated relay session ID */
    uint32_t peer_node_id;          /* Remote peer's node ID */
    struct mesh_relay_session *session;  /* Pointer to session */

    /* For the first/last hop, we use a real RDMA connection */
    void *direct_comm;              /* Underlying direct comm to first/last hop */
    int is_sender;                  /* 1 if we're the originator, 0 if receiver */
};

/*
 * Relay initialization and shutdown
 */

/* Initialize relay subsystem */
int mesh_relay_init(void);

/* Shutdown relay subsystem */
void mesh_relay_destroy(void);

/* Start relay service thread */
int mesh_relay_service_start(void);

/* Stop relay service thread */
void mesh_relay_service_stop(void);

/*
 * Relay session management
 */

/* Create a new relay session (called by originator) */
struct mesh_relay_session* mesh_relay_session_create(
    uint32_t dst_node_id,
    const struct mesh_route_entry *route);

/* Find existing session by ID */
struct mesh_relay_session* mesh_relay_session_find(uint32_t session_id);

/* Find session by src/dst pair */
struct mesh_relay_session* mesh_relay_session_find_by_peers(
    uint32_t src_node_id,
    uint32_t dst_node_id);

/* Destroy a relay session */
void mesh_relay_session_destroy(struct mesh_relay_session *session);

/*
 * Relay path setup
 */

/* Set up relay path to destination (blocking) */
int mesh_relay_setup_path(uint32_t dst_node_id, struct mesh_relay_session **session_out);

/* Handle incoming relay setup request (called by relay service) */
int mesh_relay_handle_setup_req(const struct mesh_relay_header *hdr,
                                 void *recv_comm);

/* Handle incoming relay setup acknowledgment */
int mesh_relay_handle_setup_ack(const struct mesh_relay_header *hdr);

/*
 * Relay data transfer
 */

/* Send data through relay path */
int mesh_relay_send(struct mesh_relay_session *session,
                    void *data, size_t size,
                    void **request);

/* Check if relay send completed */
int mesh_relay_send_test(void *request, int *done, int *size);

/* Receive data from relay path (called at destination) */
int mesh_relay_recv(struct mesh_relay_session *session,
                    void *data, size_t size,
                    void **request);

/* Check if relay receive completed */
int mesh_relay_recv_test(void *request, int *done, int *size);

/* Forward data to next hop (called by relay service on intermediate nodes) */
int mesh_relay_forward(struct mesh_relay_session *session,
                       const struct mesh_relay_header *hdr,
                       void *data, size_t size);

/*
 * Relay connection establishment (integrates with mesh_connect)
 */

/* Check if connection to peer requires relay */
int mesh_relay_needed(uint32_t peer_node_id);

/* Establish relay connection to non-adjacent peer */
int mesh_relay_connect(uint32_t peer_node_id, void **relay_comm);

/* Accept relay connection from non-adjacent peer */
int mesh_relay_accept(void **relay_comm);

/* Close relay connection */
int mesh_relay_close(void *relay_comm);

/*
 * Helper functions
 */

/* Get the next hop node ID for a relay path */
uint32_t mesh_relay_get_next_hop(const struct mesh_relay_session *session);

/* Get our position in the relay path (0=src, path_len-1=dst) */
int mesh_relay_get_position(const struct mesh_relay_session *session);

/* Check if we are the source of this relay session */
int mesh_relay_is_source(const struct mesh_relay_session *session);

/* Check if we are the destination of this relay session */
int mesh_relay_is_destination(const struct mesh_relay_session *session);

/* Check if we are a relay node (intermediate) */
int mesh_relay_is_intermediate(const struct mesh_relay_session *session);

/* Generate unique session ID */
uint32_t mesh_relay_generate_session_id(void);

/*
 * Relay statistics and diagnostics
 */

/* Dump relay state to log */
void mesh_relay_dump_state(void);

/* Dump active sessions to log */
void mesh_relay_dump_sessions(void);

#endif /* MESH_ROUTING_H */
