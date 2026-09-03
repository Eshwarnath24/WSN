import { initializeApp } from "https://www.gstatic.com/firebasejs/10.12.0/firebase-app.js";
import { getDatabase } from "https://www.gstatic.com/firebasejs/10.12.0/firebase-database.js";

/**
 * Parses simple KEY="VALUE" or KEY=VALUE lines from .env text
 */
function parseEnv(text) {
  const env = {};
  if (!text) return env;
  const lines = text.split("\n");
  for (const line of lines) {
    const trimmed = line.trim();
    if (!trimmed || trimmed.startsWith("#")) continue;
    const eqIdx = trimmed.indexOf("=");
    if (eqIdx !== -1) {
      const key = trimmed.slice(0, eqIdx).trim();
      let val = trimmed.slice(eqIdx + 1).trim();
      if ((val.startsWith('"') && val.endsWith('"')) || (val.startsWith("'") && val.endsWith("'"))) {
        val = val.slice(1, -1);
      }
      env[key] = val;
    }
  }
  return env;
}

let loadedEnv = {};

// Asynchronously load .env using top-level await in ES Module
try {
  const res = await fetch("./.env");
  if (res.ok) {
    const rawText = await res.text();
    loadedEnv = parseEnv(rawText);
  }
} catch (err) {
  console.warn("[Firebase Config] Could not load .env directly via fetch:", err);
}

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

  // 3. Loaded .env key or VITE_ prefixed key
  if (loadedEnv[key]) return loadedEnv[key];
  if (loadedEnv[`VITE_${key}`]) return loadedEnv[`VITE_${key}`];

  return "";
}

const firebaseConfig = {
  apiKey: getEnv("FIREBASE_API_KEY"),
  authDomain: getEnv("FIREBASE_AUTH_DOMAIN"),
  databaseURL: getEnv("FIREBASE_DATABASE_URL"),
  projectId: getEnv("FIREBASE_PROJECT_ID"),
  storageBucket: getEnv("FIREBASE_STORAGE_BUCKET"),
  messagingSenderId: getEnv("FIREBASE_MESSAGING_SENDER_ID"),
  appId: getEnv("FIREBASE_APP_ID")
};

if (!firebaseConfig.apiKey || !firebaseConfig.databaseURL) {
  console.error(
    "[Firebase Config] Missing Firebase credentials! Ensure 'dashboard/.env' exists with valid FIREBASE_* configuration values."
  );
}

const app = initializeApp(firebaseConfig);
const db = getDatabase(app);

export { db, firebaseConfig };