import { join } from 'node:path'
import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'
import cesium from 'vite-plugin-cesium'

import { VitePluginDoubleshot } from 'vite-plugin-doubleshot'

import { viteStaticCopy } from 'vite-plugin-static-copy'

// https://vitejs.dev/config/
export default defineConfig({
  assetsInclude: ['**/*.glb', 'src/render/assets/models/yildizusv.glb', 'src/render/assets/models/uav.glb'],
  envDir: __dirname,
  root: join(__dirname, 'src/render'),
  plugins: [
    react(),
    cesium(),
    viteStaticCopy({
      targets: [
        // {
        //   src: 'src/render/assets/models/uav.glb',
        //   dest: 'assets/models',
        // },
      ],
    }),
    VitePluginDoubleshot({
      type: 'electron',
      main: 'dist/main/index.js',
      entry: 'src/main/index.ts',
      outDir: 'dist/main',
      external: ['electron'],
      waitTimeout: 60000, // 1 minute
      electron: {
        build: {
          config: './electron-builder.config.js',
        },
        preload: {
          entry: 'src/preload/index.ts',
          outDir: 'dist/preload',
        },
      },
    }),
  ],
  server: {
    watch: {
      usePolling: true, 
    },
    port: 6001,
    strictPort: true,
    host: '127.0.0.1',
    open: true,
    cors: true,
    hmr: {
      overlay: false,
    },
    headers: {
      'Access-Control-Allow-Origin': '*',
    },
  },
  optimizeDeps: {
    include: ['react', 'react-dom', '@doubleshot/nest-electron'],
  },
  resolve: {
    alias: {
      '@render': join(__dirname, 'src/render'),
      '@main': join(__dirname, 'src/main'),
    },
  },
  base: './',
  build: {
    outDir: join(__dirname, 'dist/render'),
    emptyOutDir: true,
  },
})
