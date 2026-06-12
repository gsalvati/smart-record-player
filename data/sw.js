const CACHE_NAME = 'gira-v3';
const urlsToCache = [
  '/',
  '/index.html',
  '/config.html',
  '/wifi.html',
  '/style.css',
  '/style2.css',
  '/script2.js',
  '/i18n.js',
  '/manifest.json'
];

self.addEventListener('install', event => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then(cache => cache.addAll(urlsToCache))
  );
});

self.addEventListener('activate', event => {
  event.waitUntil(
    caches.keys().then(cacheNames => {
      return Promise.all(
        cacheNames.map(cacheName => {
          if (cacheName !== CACHE_NAME) {
            return caches.delete(cacheName);
          }
        })
      );
    })
  );
});

self.addEventListener('fetch', event => {
  if(event.request.method !== 'GET' || event.request.url.includes('/get_config') || event.request.url.includes('/salvar')) {
      return fetch(event.request);
  }
  event.respondWith(
    caches.match(event.request)
      .then(response => {
        if (response) {
          return response;
        }
        return fetch(event.request);
      })
  );
});
