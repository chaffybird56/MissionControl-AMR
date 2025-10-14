/** @type {import('next').NextConfig} */
const nextConfig = {
  reactStrictMode: true,
  swcMinify: true,
  env: {
    API_URL: process.env.API_URL || 'http://localhost:8002',
    WS_URL: process.env.WS_URL || 'ws://localhost:8002/ws'
  }
}

module.exports = nextConfig
