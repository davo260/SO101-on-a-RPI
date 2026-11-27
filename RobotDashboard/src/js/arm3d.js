/**
 * Visualizador Three.js modelado con las proporciones, cavidades esqueletizadas
 * y chasis exacto del brazo SO-101.
 * 
 * Esquema de Color:
 * - Brazo Consigna (target): Bitonal Verde Lima / Pistacho (#9dc938) con componentes y soportes en Negro (#181a1b).
 * - Brazo Real (telemetrÃ­a): Blanco puro (#f8fafc) con servomotores y bocinas contrastadas.
 */
function createArmViewer(container, theme = 'target') {
  const width = container.clientWidth || 300;
  const height = container.clientHeight || 300;

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
  camera.position.set(24, 20, 26);

  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(width, height);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.target.set(0, 8.5, 0);

  // IluminaciÃ³n
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
  scene.add(ambientLight);

  const keyLight = new THREE.DirectionalLight(0xffffff, 0.95);
  keyLight.position.set(20, 35, 25);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.width = 1024;
  keyLight.shadow.mapSize.height = 1024;
  scene.add(keyLight);

  const fillLight = new THREE.DirectionalLight(0x94a3b8, 0.4);
  fillLight.position.set(-20, 15, -15);
  scene.add(fillLight);

  // Rejilla de referencia
  const grid = new THREE.GridHelper(28, 28, 0x1e293b, 0x0f172a);
  grid.position.y = 0;
  scene.add(grid);

  // Materiales segÃºn tema (Target: Verde lima + Negro | Real: Blanco)
  const isTarget = theme === 'target';

  const matLime = new THREE.MeshStandardMaterial({
    color: 0x9dc938,
    roughness: 0.4,
    metalness: 0.08
  });

  const matBlackChassis = new THREE.MeshStandardMaterial({
    color: 0x181a1b,
    roughness: 0.55,
    metalness: 0.15
  });

  const matWhiteReal = new THREE.MeshStandardMaterial({
    color: 0xf8fafc,
    roughness: 0.35,
    metalness: 0.05
  });

  const matPrimary = isTarget ? matLime : matWhiteReal;
  const matSecondary = matBlackChassis;

  const matDarkServo = new THREE.MeshStandardMaterial({
    color: 0x111215,
    roughness: 0.6,
    metalness: 0.3
  });

  const matHorn = new THREE.MeshStandardMaterial({
    color: 0xd4d4d8,
    roughness: 0.25,
    metalness: 0.8
  });

  const matHighlight = new THREE.MeshBasicMaterial({
    color: isTarget ? 0xa3e635 : 0x38bdf8,
    wireframe: true
  });

  // Piezas impresas redondeadas del SO-101 y servos STS3215.
  function addBox(parent, w, h, d, material, x=0, y=0, z=0) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(w,h,d), material);
    m.position.set(x,y,z); m.castShadow=true; m.receiveShadow=true; parent.add(m); return m;
  }
  function addPlate(parent, w, len, depth, material, x, y, z, r=0.35) {
    const a=Math.min(r,w/2,len/2), sh=new THREE.Shape();
    sh.moveTo(-w/2+a,-len/2); sh.lineTo(w/2-a,-len/2);
    sh.quadraticCurveTo(w/2,-len/2,w/2,-len/2+a); sh.lineTo(w/2,len/2-a);
    sh.quadraticCurveTo(w/2,len/2,w/2-a,len/2); sh.lineTo(-w/2+a,len/2);
    sh.quadraticCurveTo(-w/2,len/2,-w/2,len/2-a); sh.lineTo(-w/2,-len/2+a);
    sh.quadraticCurveTo(-w/2,-len/2,-w/2+a,-len/2);
    const g=new THREE.ExtrudeGeometry(sh,{depth,bevelEnabled:true,bevelSegments:2,bevelSize:0.045,bevelThickness:0.04,curveSegments:8});
    g.translate(0,0,-depth/2); const m=new THREE.Mesh(g,material); m.position.set(x,y,z); m.castShadow=true; m.receiveShadow=true; parent.add(m); return m;
  }
  function addScrew(parent,x,y,z,r=0.075) {
    const m=new THREE.Mesh(new THREE.CylinderGeometry(r,r,0.075,12),matHorn);
    m.rotation.x=Math.PI/2; m.position.set(x,y,z); parent.add(m);
  }
  function createServoBlock(sx=2.0,sy=1.45,sz=1.55) {
    const g=new THREE.Group();
    addBox(g,sx,sy,sz,matDarkServo);
    addBox(g,sx*.88,.2,sz*.9,matSecondary,0,sy*.42,0);
    addBox(g,sx*.92,.12,sz*.82,matSecondary,0,-sy*.42,0);
    [-1,1].forEach(side=>{
      addBox(g,.12,sy*.72,sz*.92,matDarkServo,side*sx*.48,0,0);
      [-.4,.4].forEach(y=>{const b=new THREE.Mesh(new THREE.CylinderGeometry(.07,.07,.08,10),matHorn);b.rotation.z=Math.PI/2;b.position.set(side*(sx/2+.04),y,0);g.add(b);});
    });
    return g;
  }
  function createHornDisc(radius=.65,thickness=.18) {
    const grp=new THREE.Group();
    const disc=new THREE.Mesh(new THREE.CylinderGeometry(radius,radius,thickness,28),matHorn);disc.rotation.x=Math.PI/2;grp.add(disc);
    const c=new THREE.Mesh(new THREE.CylinderGeometry(.14,.14,thickness+.04,16),matDarkServo);c.rotation.x=Math.PI/2;grp.add(c);
    for(let i=0;i<4;i++){const a=i*Math.PI/2+Math.PI/4,b=new THREE.Mesh(new THREE.CylinderGeometry(.06,.06,thickness+.05,10),matDarkServo);b.rotation.x=Math.PI/2;b.position.set(Math.cos(a)*radius*.65,Math.sin(a)*radius*.65,0);grp.add(b);}
    return grp;
  }
  function addLink(parent,length,width,depth,material,x) {
    addPlate(parent,width,length,depth,material,x,length/2,0,.38);
    // Ventana longitudinal y tornillos visibles en las caras de las placas.
    addBox(parent,width*.22,length*.58,.035,matSecondary,x,length/2,depth/2+.025);
    [-length*.36,-length*.12,length*.12,length*.36].forEach(y=>addScrew(parent,x,length/2+y,depth/2+.06));
  }
  function addCable(parent,points,material,r=.05){
    const c=new THREE.CatmullRomCurve3(points.map(p=>new THREE.Vector3(...p)));
    parent.add(new THREE.Mesh(new THREE.TubeGeometry(c,28,r,7,false),material));
  }

  const arm = {};
  // Base de mesa con pie perforado, pedestal y primer servo alojado en el soporte.
  const baseMount=new THREE.Group();
  addPlate(baseMount,5.0,4.3,.48,matSecondary,0,.24,0,.38);
  addBox(baseMount,3.0,1.15,2.75,matPrimary,0,1.0,0);
  addBox(baseMount,2.5,.28,2.3,matSecondary,0,1.68,0);
  const baseServo=createServoBlock(2.1,1.45,1.7);baseServo.position.set(0,2.2,0);baseMount.add(baseServo);
  addBox(baseMount,1.8,.3,1.2,matSecondary,0,.72,-1.55);
  scene.add(baseMount);

  // J1: giro de base, con carcasa ancha y hombro en forma de horquilla.
  arm.shoulder_pan=new THREE.Group();arm.shoulder_pan.position.y=2.7;scene.add(arm.shoulder_pan);
  addBox(arm.shoulder_pan,2.7,.7,2.3,matSecondary,0,.15,0);
  addBox(arm.shoulder_pan,2.35,.34,1.95,matPrimary,0,.62,0);
  const panServo=createServoBlock(2.15,1.45,1.65);panServo.position.set(0,1.05,0);arm.shoulder_pan.add(panServo);

  // J2: servo de hombro lateral y brazo superior formado por dos placas paralelas.
  arm.shoulder_lift=new THREE.Group();arm.shoulder_lift.position.set(0,1.15,0);arm.shoulder_pan.add(arm.shoulder_lift);
  const shoulderServo=createServoBlock(2.15,1.5,1.65);shoulderServo.position.set(0,.2,0);arm.shoulder_lift.add(shoulderServo);
  addBox(arm.shoulder_lift,.38,1.65,2.1,matSecondary,-1.18,.1,0);addBox(arm.shoulder_lift,.38,1.65,2.1,matSecondary,1.18,.1,0);
  const shoulderHorn=createHornDisc(.76,.2);shoulderHorn.position.set(1.38,.2,.9);arm.shoulder_lift.add(shoulderHorn);
  const upper=new THREE.Group();addLink(upper,5.9,1.05,.48,matPrimary,-.62);addLink(upper,5.9,1.05,.48,matPrimary,.62);
  addBox(upper,.78,4.8,.72,matSecondary,0,3,-.08);addBox(upper,1.45,.65,1.2,matSecondary,0,5.55,0);arm.shoulder_lift.add(upper);

  // J3: codo con servo expuesto y placas dobles del antebrazo.
  arm.elbow_flex=new THREE.Group();arm.elbow_flex.position.set(0,5.9,0);arm.shoulder_lift.add(arm.elbow_flex);
  const elbowServo=createServoBlock(2.1,1.5,1.65);elbowServo.position.set(0,.05,0);arm.elbow_flex.add(elbowServo);
  addBox(arm.elbow_flex,2.7,.42,2.0,matSecondary,0,-.72,0);
  const elbowHorn=createHornDisc(.72,.2);elbowHorn.position.set(-1.3,.05,.9);arm.elbow_flex.add(elbowHorn);
  const forearm=new THREE.Group();addLink(forearm,5.15,.96,.46,matPrimary,-.57);addLink(forearm,5.15,.96,.46,matPrimary,.57);
  addBox(forearm,.78,4.1,.78,matSecondary,0,2.55,-.08);addBox(forearm,1.65,.8,1.5,matSecondary,0,4.7,0);arm.elbow_flex.add(forearm);

  // J4 y J5: muÃ±eca compacta, dos servos en soportes impresos y bocinas circulares.
  arm.wrist_flex=new THREE.Group();arm.wrist_flex.position.set(0,5.15,0);arm.elbow_flex.add(arm.wrist_flex);
  const wristFlexServo=createServoBlock(1.95,1.4,1.55);wristFlexServo.position.set(0,.15,0);arm.wrist_flex.add(wristFlexServo);
  addBox(arm.wrist_flex,2.25,.42,1.85,matPrimary,0,-.62,0);
  const wristHorn=createHornDisc(.62,.18);wristHorn.position.set(1.15,.15,.82);arm.wrist_flex.add(wristHorn);
  arm.wrist_roll=new THREE.Group();arm.wrist_roll.position.set(0,1.0,0);arm.wrist_flex.add(arm.wrist_roll);
  addBox(arm.wrist_roll,1.8,.65,1.55,matSecondary,0,.1,0);
  const rollServo=createServoBlock(1.65,1.3,1.45);rollServo.position.set(0,.85,0);arm.wrist_roll.add(rollServo);
  const rollHorn=createHornDisc(.58,.18);rollHorn.position.set(0,1.5,.82);arm.wrist_roll.add(rollHorn);

  // J6: lÃ­der con mango y gatillo manual; seguidor con pinza de una mordaza mÃ³vil.
  arm.gripper=new THREE.Group();arm.gripper.position.set(0,1.65,0);arm.wrist_roll.add(arm.gripper);
  if(isTarget){
    addBox(arm.gripper,1.8,1.15,1.45,matSecondary,0,.25,0);
    addPlate(arm.gripper,1.2,3.8,.72,matSecondary,0,-1.7,0,.45);
    addBox(arm.gripper,1.35,.55,1.0,matPrimary,0,.9,0);
    const gripServo=createServoBlock(1.2,1.25,1.2);gripServo.position.set(.78,.2,0);arm.gripper.add(gripServo);
    arm.trigger=new THREE.Group();arm.trigger.position.set(.7,.2,.72);arm.gripper.add(arm.trigger);
    addPlate(arm.trigger,.48,1.95,.42,matSecondary,0,-.7,0,.2);
    const triggerHorn=createHornDisc(.4,.16);triggerHorn.position.set(0,0,.25);arm.trigger.add(triggerHorn);
  }else{
    addPlate(arm.gripper,2.15,1.8,.85,matPrimary,0,.15,0,.3);
    addBox(arm.gripper,1.75,.95,1.25,matSecondary,0,.45,-.08);
    const gripServo=createServoBlock(1.25,1.55,1.25);gripServo.position.set(-.85,.55,0);arm.gripper.add(gripServo);
    addPlate(arm.gripper,.68,2.7,.55,matPrimary,.65,2.0,0,.25);
    addPlate(arm.gripper,.58,2.55,.5,matSecondary,-.55,1.95,0,.24);
    arm.movableJaw=new THREE.Group();arm.movableJaw.position.set(-.55,.65,.02);arm.gripper.add(arm.movableJaw);
    addPlate(arm.movableJaw,.55,1.0,.55,matPrimary,0,.35,0,.2);
    addPlate(arm.movableJaw,.58,2.45,.5,matSecondary,0,1.75,0,.22);
    const clawHorn=createHornDisc(.46,.16);clawHorn.position.set(-.55,.6,.48);arm.gripper.add(clawHorn);
  }

  // Cables rojo/blanco/negro caracterÃ­sticos, guiados por el lateral posterior del brazo.
  addCable(arm.shoulder_lift,[[-1.0,.9,-.9],[-1.4,1.8,-1.0],[-1.1,3.0,-.95],[-.9,4.6,-.9],[.1,5.6,-.85]],new THREE.MeshBasicMaterial({color:0xe32636}),.06);
  addCable(arm.shoulder_lift,[[-.9,.85,-.98],[-1.25,1.7,-1.05],[-.95,3.1,-1.0],[-.7,4.7,-.95],[.25,5.55,-.92]],new THREE.MeshBasicMaterial({color:0xf4f4f4}),.052);
  addCable(arm.shoulder_lift,[[-.8,.8,-1.05],[-1.05,1.6,-1.1],[-.75,3.2,-1.05],[-.5,4.8,-1.0],[.4,5.55,-.98]],new THREE.MeshBasicMaterial({color:0x171717}),.05);
  addCable(arm.elbow_flex,[[-.95,.1,-.95],[-1.3,1.2,-1.0],[-1.2,2.8,-.95],[-.7,4.5,-.9],[.25,5.0,-.8]],new THREE.MeshBasicMaterial({color:0xe32636}),.05);
  addCable(arm.wrist_flex,[[.75,.2,-.85],[1.15,.8,-.9],[.9,1.6,-.9],[.25,2.0,-.85]],new THREE.MeshBasicMaterial({color:0xf4f4f4}),.05);

  // Halo de selecciÃ³n
  const halo = new THREE.Mesh(new THREE.TorusGeometry(1.6, 0.08, 12, 24), matHighlight);
  halo.rotation.x = Math.PI / 2;
  halo.visible = false;
  scene.add(halo);

  window.addEventListener('resize', () => {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  });

  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }
  animate();

  return {
    /**
     * Aplica la cinemÃ¡tica en base a las variables normalizadas:
     * shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll: -100 a 100 (0 es centro)
     * gripper: 0 a 100 (0 cerrado, 100 apertura total)
     */
    setNormalizedPose(pose) {
      if (pose.shoulder_pan !== undefined) {
        const deg = (pose.shoulder_pan / 100) * 110;
        arm.shoulder_pan.rotation.y = THREE.MathUtils.degToRad(deg);
      }
      if (pose.shoulder_lift !== undefined) {
        const deg = (pose.shoulder_lift / 100) * 100;
        arm.shoulder_lift.rotation.x = THREE.MathUtils.degToRad(deg);
      }
      if (pose.elbow_flex !== undefined) {
        // PosiciÃ³n de referencia basada en las fotografÃ­as:
        // -100 = 0Â°; 0 = 90Â° (antebrazo vertical, postura de referencia);
        // +100 = 180Â° (antebrazo plegado hacia el hombro).
        const value = Math.max(-100, Math.min(100, pose.elbow_flex));
        const deg = 90 + (value / 100) * 90;
        arm.elbow_flex.rotation.x = THREE.MathUtils.degToRad(deg);
      }
      if (pose.wrist_flex !== undefined) {
        const deg = (pose.wrist_flex / 100) * 95;
        arm.wrist_flex.rotation.x = THREE.MathUtils.degToRad(deg);
      }
      if (pose.wrist_roll !== undefined) {
        // -100 a 100 mapeado a vuelta completa (-180Â° a +180Â°)
        const deg = (pose.wrist_roll / 100) * 160;
        arm.wrist_roll.rotation.y = THREE.MathUtils.degToRad(deg);
      }
      if (pose.gripper !== undefined) {
        const ratio = Math.max(0, Math.min(100, pose.gripper)) / 100;
        const gripRotZ = THREE.MathUtils.degToRad(-10 + ratio * 70);
        if (arm.movableJaw) arm.movableJaw.rotation.z = gripRotZ;
        if (arm.trigger) arm.trigger.rotation.z = -gripRotZ * 0.75;
      }
    },
    highlightJoint(jointName) {
      if (arm[jointName]) {
        halo.visible = true;
        arm[jointName].add(halo);
      } else {
        halo.visible = false;
      }
    }
  };
}