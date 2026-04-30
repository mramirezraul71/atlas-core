CREATE TABLE IF NOT EXISTS comms_messages (
    message_id TEXT PRIMARY KEY,
    created_at TEXT NOT NULL,
    user_id TEXT,
    channel TEXT,
    urgency TEXT,
    request_summary TEXT,
    request_encrypted TEXT NOT NULL,
    response_summary TEXT,
    response_encrypted TEXT,
    inventory_status TEXT,
    camera_status TEXT,
    offline_mode INTEGER NOT NULL DEFAULT 0,
    synced INTEGER NOT NULL DEFAULT 1,
    sync_attempts INTEGER NOT NULL DEFAULT 0,
    last_error TEXT,
    metadata_json TEXT
);

CREATE TABLE IF NOT EXISTS comms_offline_queue (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    message_id TEXT NOT NULL,
    payload_encrypted TEXT NOT NULL,
    created_at TEXT NOT NULL,
    status TEXT NOT NULL DEFAULT 'pending',
    attempts INTEGER NOT NULL DEFAULT 0,
    last_error TEXT,
    dequeued_at TEXT,
    FOREIGN KEY(message_id) REFERENCES comms_messages(message_id)
);

CREATE INDEX IF NOT EXISTS idx_comms_messages_created_at ON comms_messages(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_comms_messages_synced ON comms_messages(synced, created_at DESC);
CREATE INDEX IF NOT EXISTS idx_comms_queue_status ON comms_offline_queue(status, created_at ASC);

CREATE TABLE IF NOT EXISTS comms_external_users (
    source TEXT NOT NULL,
    external_user_id TEXT NOT NULL,
    username TEXT,
    full_name TEXT,
    email TEXT,
    role TEXT,
    status TEXT,
    whatsapp_number TEXT,
    whatsapp_number_norm TEXT,
    whatsapp_state TEXT NOT NULL DEFAULT 'missing',
    requested_at TEXT,
    linked_at TEXT,
    updated_at TEXT NOT NULL,
    requested_by TEXT,
    request_note TEXT,
    meta_json TEXT,
    PRIMARY KEY(source, external_user_id)
);

CREATE INDEX IF NOT EXISTS idx_comms_external_users_state
ON comms_external_users(whatsapp_state, source, updated_at DESC);
CREATE INDEX IF NOT EXISTS idx_comms_external_users_phone
ON comms_external_users(whatsapp_number_norm);
