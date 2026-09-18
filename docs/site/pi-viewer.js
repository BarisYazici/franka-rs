(function () {
  'use strict';
  const viewport = document.getElementById('viewer');
  const poster = document.getElementById('poster');
  const status = document.getElementById('status');
  const download = document.getElementById('download');
  const models = {
    enclosure: {file: 'enclosure-scene.glb', image: 'enclosure.png', label: 'Enclosure'},
    assembly: {file: 'assembly.glb', image: 'assembly.png', label: 'Electronics'},
    exploded: {file: 'assembly-exploded.glb', image: 'assembly-exploded.png', label: 'Exploded electronics'}
  };
  const selected = new URLSearchParams(location.search).get('model');
  let current = models[selected] ? selected : 'enclosure';
  let renderer, scene, camera, controls, root, generation = 0;
  let center;
  let radius = 0.2;
  let contextLost = false;

  function selectLabel(name) {
    current = name;
    document.querySelectorAll('[data-model]').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.model === name)));
    download.href = '../assets/pi5/' + models[name].file;
    poster.src = '../assets/pi5/' + models[name].image;
    poster.alt = models[name].label + ' reference CAD illustration.';
  }
  function render() { renderer.render(scene, camera); }
  function reset() {
    const distance = radius / Math.sin(THREE.MathUtils.degToRad(camera.fov / 2)) * 1.25;
    camera.position.copy(center).add(new THREE.Vector3(1.2, 0.85, 1.3).normalize().multiplyScalar(distance));
    controls.target.copy(center);
    camera.near = 0.001;
    camera.far = Math.max(10, distance * 20);
    camera.updateProjectionMatrix();
    controls.update();
    render();
  }
  function dispose(model) {
    if (!model) return;
    const materials = new Set();
    model.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) (Array.isArray(o.material) ? o.material : [o.material]).forEach(m => materials.add(m)); });
    materials.forEach(m => m.dispose());
  }
  async function load(name) {
    if (contextLost) return;
    const token = ++generation;
    selectLabel(name);
    status.textContent = 'Loading ' + models[name].label.toLowerCase() + '…';
    try {
      const gltf = await new THREE.GLTFLoader().loadAsync('../assets/pi5/' + models[name].file);
      if (token !== generation) { dispose(gltf.scene); return; }
      if (root) { scene.remove(root); dispose(root); }
      root = gltf.scene;
      scene.add(root);
      root.updateMatrixWorld(true);
      const bounds = new THREE.Box3();
      // Frame the electronics, not the long illustrative cables leaving the model.
      const focus = name === 'enclosure' ? ['enc_base', 'enc_lid'] : ['pi5', 'active_cooler', 'pcie_adapter', 'i350_t2'];
      focus.forEach(n => { const object = root.getObjectByName(n); if (object) bounds.expandByObject(object); });
      if (bounds.isEmpty()) bounds.setFromObject(root);
      bounds.getCenter(center);
      radius = bounds.getSize(new THREE.Vector3()).length() / 2;
      poster.hidden = true;
      renderer.domElement.hidden = false;
      reset();
      status.textContent = models[name].label + ' ready. Geometry is illustrative; verify fit against your hardware.';
    } catch (error) {
      if (token !== generation) return;
      poster.hidden = false;
      renderer.domElement.hidden = true;
      status.textContent = 'The 3D model could not load. The illustration and downloads on the guide page remain available.';
    }
  }
  selectLabel(current);
  try {
    center = new THREE.Vector3();
    scene = new THREE.Scene();
    scene.background = new THREE.Color('#ece9e2');
    camera = new THREE.PerspectiveCamera(40, 1, 0.001, 10);
    renderer = new THREE.WebGLRenderer({antialias: true});
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.outputEncoding = THREE.sRGBEncoding;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.35;
    const canvas = renderer.domElement;
    canvas.hidden = true;
    canvas.tabIndex = 0;
    canvas.setAttribute('role', 'application');
    canvas.setAttribute('aria-label', 'Interactive Pi controller model. Arrow keys rotate; plus and minus zoom; R resets.');
    viewport.appendChild(canvas);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x666055, 1.5));
    const key = new THREE.DirectionalLight(0xffffff, 2.4); key.position.set(1, 2, 3); scene.add(key);
    const fill = new THREE.DirectionalLight(0xffffff, 1.2); fill.position.set(-2, 1, -1); scene.add(fill);
    controls = new THREE.OrbitControls(camera, canvas);
    controls.minDistance = 0.04;
    controls.maxDistance = 2;
    controls.addEventListener('change', render);
    function resize() {
      const width = viewport.clientWidth, height = viewport.clientHeight;
      renderer.setSize(width, height);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      render();
    }
    new ResizeObserver(resize).observe(viewport);
    resize();
    canvas.addEventListener('keydown', event => {
      const keys = ['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','=','-','r','R'];
      if (!keys.includes(event.key)) return;
      event.preventDefault();
      if (event.key.toLowerCase() === 'r') { reset(); return; }
      const sphere = new THREE.Spherical().setFromVector3(camera.position.clone().sub(controls.target));
      if (event.key === 'ArrowLeft') sphere.theta -= 0.15;
      if (event.key === 'ArrowRight') sphere.theta += 0.15;
      if (event.key === 'ArrowUp') sphere.phi -= 0.15;
      if (event.key === 'ArrowDown') sphere.phi += 0.15;
      if (event.key === '+' || event.key === '=') sphere.radius *= 0.9;
      if (event.key === '-') sphere.radius *= 1.1;
      sphere.makeSafe();
      sphere.radius = Math.max(0.04, Math.min(2, sphere.radius));
      camera.position.copy(controls.target).add(new THREE.Vector3().setFromSpherical(sphere));
      controls.update(); render();
    });
    canvas.addEventListener('webglcontextlost', event => {
      event.preventDefault(); contextLost = true; ++generation; poster.hidden = false; canvas.hidden = true;
      status.textContent = '3D rendering was interrupted. Reload the page to retry, or use the downloads.';
    });
    document.querySelectorAll('[data-model]').forEach(button => button.addEventListener('click', () => load(button.dataset.model)));
    document.getElementById('reset').addEventListener('click', reset);
    load(current);
  } catch (error) {
    status.textContent = 'Interactive 3D is unavailable in this browser. Use the illustration and downloads below.';
    document.querySelectorAll('[data-model]').forEach(b => b.addEventListener('click', () => selectLabel(b.dataset.model)));
    document.getElementById('reset').disabled = true;
  }
})();
