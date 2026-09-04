import { initializeApp } from "https://www.gstatic.com/firebasejs/10.12.0/firebase-app.js";
import { getDatabase } from "https://www.gstatic.com/firebasejs/10.12.0/firebase-database.js";

/**
 * Firebase Project Configuration
 * Default credentials for the ESP32 Wireless Sensor Network project.
 * Can be overridden via window.__ENV__ or Vite environment variables.
 */
const DEFAULT_CONFIG = {
  apiKey: "AIzaSyAgiQYFCgjkdKbU5VqQQHHLQCvAldQTW2M",
  authDomain: "wsn-project-bb012.firebaseapp.com",
  databaseURL: "https://wsn-project-bb012-default-rtdb.asia-southeast1.firebasedatabase.app",
  projectId: "wsn-project-bb012",
  storageBucket: "wsn-project-bb012.firebasestorage.app",
  messagingSenderId: "805607252",
  appId: "1:805607252:web:4383ea3054a5e183071ac1"
};

function getEnv(key) {
  // 1. Bundler environment (Vite / Rollup / Webpack)
  try {
    if (typeof import.meta !== "undefined" && import.meta.env) {
      if (import.meta.env[key]) return import.meta.env[key];
      if (import.meta.env[`VITE_${key}`]) return import.meta.env[`VITE_${key}`];
    }
  } catch (e) {}

  // 2. Global window.__ENV__ runtime injection
  if (typeof window !== "undefined" && window.__ENV__ && window.__ENV__[key]) {
    return window.__ENV__[key];
  }

  return "";
}

const firebaseConfig = {
  apiKey: getEnv("FIREBASE_API_KEY") || DEFAULT_CONFIG.apiKey,
  authDomain: getEnv("FIREBASE_AUTH_DOMAIN") || DEFAULT_CONFIG.authDomain,
  databaseURL: getEnv("FIREBASE_DATABASE_URL") || DEFAULT_CONFIG.databaseURL,
  projectId: getEnv("FIREBASE_PROJECT_ID") || DEFAULT_CONFIG.projectId,
  storageBucket: getEnv("FIREBASE_STORAGE_BUCKET") || DEFAULT_CONFIG.storageBucket,
  messagingSenderId: getEnv("FIREBASE_MESSAGING_SENDER_ID") || DEFAULT_CONFIG.messagingSenderId,
  appId: getEnv("FIREBASE_APP_ID") || DEFAULT_CONFIG.appId
};

if (!firebaseConfig.apiKey || !firebaseConfig.databaseURL) {
  console.error(
    "[Firebase Config] Missing Firebase credentials! Ensure valid FIREBASE_* configuration values are set."
  );
}

const app = initializeApp(firebaseConfig);
const db = getDatabase(app);

console.log("[Firebase Config] Firebase initialized successfully with database:", firebaseConfig.databaseURL);

export { db, firebaseConfig };