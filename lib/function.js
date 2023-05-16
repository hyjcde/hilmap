import * as THREE from 'three';

function decode64(inbytes, outbytes, record_size, pointRatio) {
  var x,b=0,l=0,j=0,L=inbytes.length,A=outbytes.length;
  record_size = record_size || A; // default copies everything (no skipping)
  pointRatio = pointRatio || 1; // default copies everything (no skipping)
  var bitskip = (pointRatio-1) * record_size * 8;
  for(x=0;x<L&&j<A;x++){
      b=(b<<6)+decode64.e[inbytes.charAt(x)];
      l+=6;
      if(l>=8){
          l-=8;
          outbytes[j++]=(b>>>l)&0xff;
          if((j % record_size) === 0) { // skip records
              // no    optimization: for(var i=0;i<bitskip;x++){l+=6;if(l>=8) {l-=8;i+=8;}}
              // first optimization: for(;l<bitskip;l+=6){x++;} l=l%8;
              x += Math.ceil((bitskip - l) / 6);
              l = l % 8;

              if(l>0){b=decode64.e[inbytes.charAt(x)];}
          }
      }
  }
  return Math.floor(j/record_size);
}

// initialize decoder with static lookup table 'e'
decode64.S='ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';
decode64.e={};
for(var i=0;i<64;i++){decode64.e[decode64.S.charAt(i)]=i;}

export class PointCloud2 extends THREE.Object3D {

  /**
   * A PointCloud2 client that listens to a given topic and displays the points.
   *
   * @constructor
   * @param options - object with following keys:
   *
   *  * ros - the ROSLIB.Ros connection handle
   *  * topic - the marker topic to listen to (default: '/points')
   *  * tfClient - the TF client handle to use
   *  * compression (optional) - message compression (default: 'cbor')
   *  * rootObject (optional) - the root object to add this marker to use for the points.
   *  * max_pts (optional) - number of points to draw (default: 10000)
   *  * pointRatio (optional) - point subsampling ratio (default: 1, no subsampling)
   *  * messageRatio (optional) - message subsampling ratio (default: 1, no subsampling)
   *  * material (optional) - a material object or an option to construct a PointsMaterial.
   *  * colorsrc (optional) - the field to be used for coloring (default: 'rgb')
   *  * colormap (optional) - function that turns the colorsrc field value to a color
   */
  constructor(options) {
    super();
    options = options || {};
    this.throttle_rate = options.throttle_rate || null;
    this.max_pts = options.max_pts || 10000;
    this.points = new Points(options);
    this.buffer = null;
  };

  processMessage(msg){
    if(!this.points.setup(msg.header.frame_id, msg.point_step, msg.fields)) {
        return;
    }

    var n, pointRatio = this.points.pointRatio;
    var bufSz = this.max_pts * msg.point_step;

    if (msg.data.buffer) {
      this.buffer = msg.data.slice(0, Math.min(msg.data.byteLength, bufSz));
       n = Math.min(msg.height*msg.width / pointRatio, this.points.positions.array.length / 3);
    } else {
      if (!this.buffer || this.buffer.byteLength < bufSz) {
        this.buffer = new Uint8Array(bufSz);
      }
      n = decode64(msg.data, this.buffer, msg.point_step, pointRatio);
      pointRatio = 1;
    }

    var dv = new DataView(this.buffer.buffer);
    var littleEndian = !msg.is_bigendian;
    var x = this.points.fields.x.offset;
    var y = this.points.fields.y.offset;
    var z = this.points.fields.z.offset;
    var base, color;
    for(var i = 0; i < n; i++){
      base = i * pointRatio * msg.point_step;
      this.points.positions.array[3*i    ] = dv.getFloat32(base+x, littleEndian);
      this.points.positions.array[3*i + 1] = dv.getFloat32(base+y, littleEndian);
      this.points.positions.array[3*i + 2] = dv.getFloat32(base+z, littleEndian);

      if(this.points.colors){
          color = this.points.colormap(this.points.getColor(dv,base,littleEndian));
          this.points.colors.array[3*i    ] = color.r;
          this.points.colors.array[3*i + 1] = color.g;
          this.points.colors.array[3*i + 2] = color.b;
      }
    }
    this.points.update(n);
  };
}


export class Points extends THREE.Object3D {

  /**
   * A set of points. Used by PointCloud2 and LaserScan.
   *
   * @constructor
   * @param options - object with following keys:
   *
   *  * tfClient - the TF client handle to use
   *  * rootObject (optional) - the root object to add this marker to use for the points.
   *  * max_pts (optional) - number of points to draw (default: 10000)
   *  * pointRatio (optional) - point subsampling ratio (default: 1, no subsampling)
   *  * messageRatio (optional) - message subsampling ratio (default: 1, no subsampling)
   *  * material (optional) - a material object or an option to construct a PointsMaterial.
   *  * colorsrc (optional) - the field to be used for coloring (default: 'rgb')
   *  * colormap (optional) - function that turns the colorsrc field value to a color
   */
  constructor(options) {
    super();
    options = options || {};
    this.rootObject = options.rootObject || new THREE.Object3D();
    this.max_pts = options.max_pts || 10000;
    this.pointRatio = options.pointRatio || 1;
    this.messageRatio = options.messageRatio || 1;
    this.messageCount = 0;
    this.material = options.material || {};
    this.colorsrc = options.colorsrc;
    this.colormap = options.colormap;

    if(('color' in options) || ('size' in options) || ('texture' in options)) {
        console.warn(
          'toplevel "color", "size" and "texture" options are deprecated.' +
          'They should beprovided within a "material" option, e.g. : '+
          ' { tfClient, material : { color: mycolor, size: mysize, map: mytexture }, ... }'
        );
    }

    this.sn = null;
  };


  setup(frame, point_step, fields)
  {
      if(this.sn===null){

        console.log("in here");
          // turn fields to a map
          fields = fields || [];
          this.fields = {};
          for(var i=0; i<fields.length; i++) {
              this.fields[fields[i].name] = fields[i];
          }
          this.geom = new THREE.BufferGeometry();

          this.positions = new THREE.BufferAttribute( new Float32Array( this.max_pts * 3), 3, false );
          this.geom.setAttribute( 'position', this.positions.setDynamic(true) );

          if(!this.colorsrc && this.fields.rgb) {
              this.colorsrc = 'rgb';
          }
          if(this.colorsrc) {
              var field = this.fields[this.colorsrc];
              if (field) {
                  this.colors = new THREE.BufferAttribute( new Float32Array( this.max_pts * 3), 3, false );
                  this.geom.setAttribute( 'color', this.colors.setDynamic(true) );
                  var offset = field.offset;
                  this.getColor = [
                      function(dv,base,le){return dv.getInt8(base+offset,le);},
                      function(dv,base,le){return dv.getUint8(base+offset,le);},
                      function(dv,base,le){return dv.getInt16(base+offset,le);},
                      function(dv,base,le){return dv.getUint16(base+offset,le);},
                      function(dv,base,le){return dv.getInt32(base+offset,le);},
                      function(dv,base,le){return dv.getUint32(base+offset,le);},
                      function(dv,base,le){return dv.getFloat32(base+offset,le);},
                      function(dv,base,le){return dv.getFloat64(base+offset,le);}
                  ][field.datatype-1];
                  this.colormap = this.colormap || function(x){return new THREE.Color(x);};
              } else {
                  console.warn('unavailable field "' + this.colorsrc + '" for coloring.');
              }
          }

          if(!this.material.isMaterial) { // if it is an option, apply defaults and pass it to a PointsMaterial
              if(this.colors && this.material.vertexColors === undefined) {
                  this.material.vertexColors = THREE.VertexColors;
              }
              this.material = new THREE.PointsMaterial(this.material);
          }

          this.object = new THREE.Points( this.geom, this.material );

          this.rootObject.add(this.object);

          this.sn = 1;
      }
      return (this.messageCount++ % this.messageRatio) === 0;
  };

  update(n)
  {
    this.geom.setDrawRange(0,n);

    this.positions.needsUpdate = true;
    this.positions.updateRange.count = n * this.positions.itemSize;

    if (this.colors) {
      this.colors.needsUpdate = true;
      this.colors.updateRange.count = n * this.colors.itemSize;
    }
  };
}