#!/usr/bin/env node
const fs = require('fs');

const versionPropertyPrefixes = ['firmware', 'deviceProtocol', 'moduleProtocol', 'userConfig', 'hardwareConfig'];
const patchVersions = ['Major', 'Minor', 'Patch'];
const package = JSON.parse(fs.readFileSync(`${__dirname}/package.json`));

const versionVariables = versionPropertyPrefixes.map(versionPropertyPrefix => {
    const versionPropertyName = `${versionPropertyPrefix}Version`
    const versionPropertyValues = package[versionPropertyName].split('.');
    return patchVersions.map(patchVersion => {
        const versionPropertyValue = versionPropertyValues.shift();
        const versionPropertyMacroName = `${versionPropertyPrefix}${patchVersion}Version`.split(/(?=[A-Z])/).join('_').toUpperCase()
        return `    #define ${versionPropertyMacroName} ${versionPropertyValue}`;
    }).join('\n') + '\n';
}).join('\n');

fs.writeFileSync(`${__dirname}/../shared/versions.h`,
`// Please do not edit this file by hand!
// It is to be regenerated by /scripts/generate-versions-h.js

#ifndef __VERSIONS_H__
#define __VERSIONS_H__

// Includes:

    #include "fsl_common.h"

// Typedefs:

    typedef struct {
        uint16_t major;
        uint16_t minor;
        uint16_t patch;
    } version_t;

// Variables:

${versionVariables}
#endif
`);
